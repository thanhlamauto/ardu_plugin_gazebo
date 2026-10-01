#!/usr/bin/env python3
"""RGB-only metric inference + Gazebo cloud publication; optional depth GT evaluation.

Latest-image queue, acquisition stamps preserved. No ground-truth scaling.
This experimental backend does not certify unobserved space as free.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import signal
import threading
import time
import numpy as np
from PIL import Image
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mppi_ardupilot.monocular_depth import depth_to_points, depth_metrics


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


def decode_image(msg, depth=False):
    # Respect padded row strides. Gazebo R_FLOAT32 is little-endian on this host.
    from gz.msgs10 import image_pb2
    expected = image_pb2.R_FLOAT32 if depth else image_pb2.RGB_INT8
    if msg.pixel_format_type != expected:
        raise ValueError(f'unsupported pixel format {msg.pixel_format_type}, expected {expected}')
    dtype, channels = ('<f4', 1) if depth else ('u1', 3)
    row_bytes = msg.width * channels * np.dtype(dtype).itemsize
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return np.frombuffer(rows[:, :row_bytes].copy().tobytes(), dtype=dtype).reshape(
        (msg.height, msg.width) if depth else (msg.height, msg.width, 3))


def cloud_message(points, header, frame_id="sensor_suite_link"):
    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
    cloud = PointCloudPacked()
    cloud.header.CopyFrom(header)
    # Image header may carry unrelated metadata. The output coordinates are FLU.
    del cloud.header.data[:]
    entry = cloud.header.data.add(key='frame_id')
    entry.value.append(frame_id)
    cloud.height, cloud.width = 1, len(points)
    cloud.point_step, cloud.row_step = 12, 12 * len(points)
    cloud.is_bigendian, cloud.is_dense = False, True
    for i, name in enumerate(('x', 'y', 'z')):
        cloud.field.add(name=name, offset=4*i, datatype=6, count=1)
    cloud.data = points.astype('<f4').tobytes()
    return cloud


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf')
    parser.add_argument('--revision', default='2fd93bd764b15eea94dcf7763bba7ddc25007d0f')
    parser.add_argument('--device', choices=['cpu', 'mps', 'cuda'], default='cpu')
    parser.add_argument('--rgb-topic', default='/sensor_suite/rgb')
    parser.add_argument('--gt-topic', default='', help='Evaluation only, e.g. /sensor_suite/depth')
    parser.add_argument('--output-topic', default='/perception/obstacles_camera')
    parser.add_argument('--output-dir', default='results/monocular_baseline')
    parser.add_argument('--frames', type=int, default=20)
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--input-size', type=int, default=518)
    parser.add_argument('--max-hz', type=float, default=15.)
    parser.add_argument('--semantic-sky', action='store_true')
    parser.add_argument('--local-map', action='store_true')
    parser.add_argument('--planning-altitude', type=float, default=3.)
    parser.add_argument('--map-lifetime', type=float, default=3.)
    parser.add_argument('--save-every', type=int, default=0, help='Save additional RGB/depth snapshots')
    parser.add_argument('--max-age', type=float, default=1., help='Maximum SIMULATION age for publishing')
    args = parser.parse_args()
    if args.frames < 1 or args.timeout <= 0 or args.max_age <= 0 or args.max_hz <= 0 or args.input_size < 14:
        parser.error('frames, timeout and max-age must be positive')
    import torch
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation, AutoModelForSemanticSegmentation
    import gz.transport13 as transport
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
    torch.set_num_threads(1)
    processor = AutoImageProcessor.from_pretrained(args.model, revision=args.revision)
    model = AutoModelForDepthEstimation.from_pretrained(args.model, revision=args.revision).to(args.device).eval()
    segmenter = segment_processor = None
    if args.semantic_sky:
        semantic_id='nvidia/segformer-b0-finetuned-ade-512-512'
        semantic_rev='489d5cd81a0b59fab9b7ea758d3548ebe99677da'
        segment_processor=AutoImageProcessor.from_pretrained(semantic_id,revision=semantic_rev)
        segmenter=AutoModelForSemanticSegmentation.from_pretrained(semantic_id,revision=semantic_rev).to(args.device).eval()
        sky_id=next(int(k) for k,v in segmenter.config.id2label.items() if v=='sky')
    local_map = None
    if args.local_map:
        from mppi_ardupilot.camera_local_map import CameraLocalMap
        from mppi_ardupilot.lidar_preprocess import quat_to_rot
        local_map = CameraLocalMap(altitude=args.planning_altitude,lifetime=args.map_lifetime)
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    node = transport.Node()
    publisher = node.advertise(args.output_topic, PointCloudPacked)
    lock = threading.Lock()
    latest, depths, sim_now = [None], deque(maxlen=60), [None]
    received = [0]
    poses = deque(maxlen=200)
    def odom_callback(msg):
        with lock:
            poses.append(msg)
    def rgb_callback(msg):
        with lock:
            received[0] += 1
            latest[0] = (msg, time.monotonic(), received[0])
    def gt_callback(msg):
        with lock:
            depths.append(msg)
    def clock_callback(msg):
        with lock:
            sim_now[0] = msg.sim.sec + msg.sim.nsec*1e-9
    if not node.subscribe(GzImage, args.rgb_topic, rgb_callback):
        raise RuntimeError('RGB subscription failed')
    node.subscribe(Clock, '/clock', clock_callback)
    if args.local_map:
        node.subscribe(Odometry, '/iris/odometry', odom_callback)
    if args.gt_topic:
        node.subscribe(GzImage, args.gt_topic, gt_callback)
    records = []
    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_requested.set())
    started = time.monotonic()
    previous_number = 0
    last_processed = -float('inf')
    with (out/'frames.jsonl').open('w') as log:
        while not stop_requested.is_set() and len(records) < args.frames and time.monotonic()-started < args.timeout:
            if time.monotonic()-last_processed < 1/args.max_hz:
                time.sleep(.01)
                continue
            with lock:
                pending, latest[0] = latest[0], None
            if pending is None:
                time.sleep(.01)
                continue
            last_processed = time.monotonic()
            msg, received_at, number = pending
            rgb = decode_image(msg)
            begin = time.monotonic()
            inputs = processor(images=Image.fromarray(rgb), return_tensors='pt',
                size={'height':args.input_size,'width':args.input_size}).to(args.device)
            with torch.inference_mode():
                prediction = model(**inputs).predicted_depth
                prediction = torch.nn.functional.interpolate(prediction[:, None], size=rgb.shape[:2],
                    mode='bicubic', align_corners=False)[0, 0].cpu().numpy()
            sky_fraction = 0.
            if segmenter is not None:
                seg_inputs=segment_processor(images=Image.fromarray(rgb),return_tensors='pt').to(args.device)
                with torch.inference_mode():
                    labels=torch.nn.functional.interpolate(segmenter(**seg_inputs).logits,size=rgb.shape[:2],mode='bilinear',align_corners=False).argmax(1)[0].cpu().numpy()
                prediction=prediction.copy(); prediction[labels==sky_id]=np.nan
                sky_fraction=float(np.mean(labels==sky_id))
            points = depth_to_points(prediction)
            inference_ms = (time.monotonic()-begin)*1000
            with lock:
                pose=min(poses,key=lambda p:abs(stamp(p)-stamp(msg))) if poses else None
                now = sim_now[0]
                gt = min(depths, key=lambda d: abs(stamp(d)-stamp(msg))) if depths else None
            age = None if now is None else now-stamp(msg)
            fresh = age is not None and 0 <= age <= args.max_age
            frame_id='sensor_suite_link'
            pose_error=None
            if local_map is not None:
                pose_error=abs(stamp(pose)-stamp(msg)) if pose else None
                if pose is None or pose_error>.06:
                    fresh=False
                elif fresh:
                    p,q=pose.pose.position,pose.pose.orientation
                    points=local_map.update(points,np.array([p.x,p.y,p.z]),quat_to_rot(q.x,q.y,q.z,q.w),stamp(msg))
                    frame_id='odom'
            # Warmup / lack of supported occupied voxels does not authorize empty-free output.
            if local_map is not None and len(points)==0:
                fresh=False
            published = bool(publisher.publish(cloud_message(points, msg.header,frame_id))) if fresh else False
            record = dict(frame=len(records), source_stamp_s=stamp(msg), inference_ms=inference_ms,
                receive_to_publish_ms=(time.monotonic()-received_at)*1000, simulation_age_s=age,
                published=published, frame_id=frame_id, pose_time_difference_s=pose_error, sky_fraction=sky_fraction, points=len(points), dropped_rgb=number-previous_number-1)
            previous_number = number
            if gt is not None and abs(stamp(gt)-stamp(msg)) <= .06:
                reference = decode_image(gt, depth=True)
                resized = torch.nn.functional.interpolate(torch.from_numpy(prediction)[None,None],
                    size=reference.shape, mode='bilinear', align_corners=False)[0,0].numpy()
                record.update(depth_metrics(resized, reference))
                record['gt_time_difference_s'] = abs(stamp(gt)-stamp(msg))
                if not records:
                    np.save(out/'ground_truth.npy', reference)
            if not records:
                Image.fromarray(rgb).save(out/'rgb.png')
                np.save(out/'predicted_depth.npy', prediction)
                np.save(out/'obstacles_sensor_flu.npy', points)
            if args.save_every and len(records) % args.save_every == 0:
                Image.fromarray(rgb).save(out/f'rgb_{len(records):06d}.png')
                np.save(out/f'depth_{len(records):06d}.npy', prediction)
            records.append(record)
            log.write(json.dumps(record, allow_nan=False)+'\n')
            log.flush()
            print(json.dumps(record), flush=True)
    if not records:
        raise SystemExit('No RGB frames received; check Gazebo and GZ_PARTITION')
    summary = dict(model=args.model, revision=args.revision, device=args.device, frames=len(records),
        published_frames=sum(r['published'] for r in records),
        inference_ms={f'p{p}':float(np.percentile([r['inference_ms'] for r in records],p)) for p in (50,95,99)},
        receive_to_publish_ms={f'p{p}':float(np.percentile([r['receive_to_publish_ms'] for r in records],p)) for p in (50,95,99)},
        latency_note='Receive-to-publish uses host monotonic clock; simulation_age_s uses acquisition and /clock. Warmup included.',
        coverage='Forward camera only; unknown space is not certified free; unknown space requires a separate coverage policy.')
    steady = records[2:]
    summary['warmup_excluded_frames'] = min(2, len(records))
    if steady:
        summary['steady_receive_to_publish_ms'] = {f'p{p}':float(np.percentile(
            [r['receive_to_publish_ms'] for r in steady],p)) for p in (50,95,99)}
    for key in ('abs_rel','rmse_m','delta1'):
        values = [r[key] for r in records if key in r]
        if values:
            summary[key] = float(np.mean(values))
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')


if __name__ == '__main__':
    main()
