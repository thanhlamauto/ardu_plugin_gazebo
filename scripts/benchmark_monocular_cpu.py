#!/usr/bin/env python3
"""Measure the current RGB-to-depth pipeline on saved Gazebo camera frames.

The replay is paced to the camera rate. Model loading and warmup are excluded
from CPU and latency statistics. Gazebo transport, PNG decoding, and cloud
publication are excluded; the same image processor, model, resize, and
depth-to-points conversion as monocular_depth_gz.py are included.
"""

import argparse
import json
import os
from pathlib import Path
import platform
import sys
import threading
import time

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mppi_ardupilot.monocular_depth import depth_to_points


DEFAULT_MODEL = 'depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf'
DEFAULT_REVISION = '2fd93bd764b15eea94dcf7763bba7ddc25007d0f'


def percentiles(values):
    return {f'p{p}': float(np.percentile(values, p)) for p in (50, 95, 99)}


def cpu_seconds(process):
    times = process.cpu_times()
    return times.user + times.system


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image-dir', type=Path, required=True,
                        help='Directory containing rgb_*.png from a saved trial')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', choices=('cpu', 'mps', 'cuda'), required=True)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--rate-hz', type=float, default=10.)
    parser.add_argument('--warmup', type=int, default=5)
    parser.add_argument('--threads', type=int, default=1)
    parser.add_argument('--input-size', type=int, default=518)
    parser.add_argument('--sample-interval', type=float, default=.2)
    parser.add_argument('--model', default=DEFAULT_MODEL)
    parser.add_argument('--revision', default=DEFAULT_REVISION)
    args = parser.parse_args()
    if (args.frames < 10 or args.warmup < 1 or args.rate_hz <= 0 or
            args.threads < 1 or args.input_size < 14 or args.sample_interval <= 0):
        parser.error('invalid frames, warmup, rate, threads, input size, or sample interval')
    paths = sorted(args.image_dir.glob('rgb_*.png'))[:args.frames]
    if len(paths) < args.frames:
        parser.error(f'need {args.frames} saved RGB frames; found {len(paths)}')

    import psutil
    import torch
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation

    if args.device == 'mps' and not torch.backends.mps.is_available():
        parser.error('MPS is unavailable')
    if args.device == 'cuda' and not torch.cuda.is_available():
        parser.error('CUDA is unavailable')
    torch.set_num_threads(args.threads)
    processor = AutoImageProcessor.from_pretrained(args.model, revision=args.revision)
    model = AutoModelForDepthEstimation.from_pretrained(
        args.model, revision=args.revision).to(args.device).eval()
    images = []
    for path in paths:
        with Image.open(path) as source:
            images.append(source.convert('RGB').copy())
    sizes = {image.size for image in images}
    if len(sizes) != 1:
        parser.error(f'camera frame sizes differ: {sizes}')

    def infer(image):
        inputs = processor(images=image, return_tensors='pt',
                           size={'height': args.input_size,
                                 'width': args.input_size}).to(args.device)
        with torch.inference_mode():
            prediction = model(**inputs).predicted_depth
            prediction = torch.nn.functional.interpolate(
                prediction[:, None], size=(image.height, image.width),
                mode='bicubic', align_corners=False)[0, 0].cpu().numpy()
        return len(depth_to_points(prediction))

    for image in images[:args.warmup]:
        infer(image)

    process = psutil.Process(os.getpid())
    stop = threading.Event()
    samples = []
    def sample_cpu():
        previous_wall = time.monotonic()
        previous_cpu = cpu_seconds(process)
        while not stop.wait(args.sample_interval):
            now = time.monotonic()
            used = cpu_seconds(process)
            samples.append(dict(wall_s=now,
                                cpu_percent_one_core=100 * (used - previous_cpu) /
                                                     (now - previous_wall),
                                rss_mb=process.memory_info().rss / 1e6))
            previous_wall, previous_cpu = now, used

    latencies = []
    point_counts = []
    late_starts = 0
    cpu_start = cpu_seconds(process)
    started = time.monotonic()
    sampler = threading.Thread(target=sample_cpu, daemon=True)
    sampler.start()
    try:
        for index, frame in enumerate(images):
            deadline = started + index / args.rate_hz
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            frame_start = time.monotonic()
            if frame_start - deadline > 1 / args.rate_hz:
                late_starts += 1
            point_counts.append(infer(frame))
            latencies.append((time.monotonic() - frame_start) * 1000)
        remaining = started + len(images) / args.rate_hz - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    finally:
        stop.set()
        sampler.join()
    elapsed = time.monotonic() - started
    used = cpu_seconds(process) - cpu_start
    logical_cores = psutil.cpu_count(logical=True)
    result = dict(
        workload='saved RGB replay, paced, no Gazebo/SITL/planner',
        included='image processor, model inference, depth resize, depth_to_points',
        excluded='model load/warmup, PNG decode, Gazebo image decode/transport, cloud publish, planner',
        host=dict(platform=platform.platform(), machine=platform.machine(),
                  logical_cpu_count=logical_cores, python=sys.version.split()[0],
                  torch=torch.__version__, transformers=__import__('transformers').__version__,
                  psutil=psutil.__version__),
        model=args.model, revision=args.revision, device=args.device,
        device_name=(torch.cuda.get_device_name() if args.device == 'cuda' else args.device),
        input_size=args.input_size, camera_image_size=list(images[0].size),
        threads=args.threads, target_rate_hz=args.rate_hz, warmup_frames=args.warmup,
        frames=len(images), elapsed_s=elapsed, achieved_rate_hz=len(images)/elapsed,
        cpu_time_s=used,
        cpu_percent_one_core=100 * used / elapsed,
        cpu_percent_all_logical_cores=100 * used / elapsed / logical_cores,
        sampled_cpu_percent_one_core=percentiles([s['cpu_percent_one_core'] for s in samples]) if samples else None,
        rss_mb=dict(start=float(samples[0]['rss_mb']) if samples else None,
                    peak=float(max(s['rss_mb'] for s in samples)) if samples else None),
        pipeline_latency_ms=percentiles(latencies),
        frames_over_period=sum(ms > 1000 / args.rate_hz for ms in latencies),
        starts_over_one_period_late=late_starts,
        points_per_frame=percentiles(point_counts),
        sample_interval_s=args.sample_interval,
        image_source=str(args.image_dir),
        measurement_note=('100% process CPU means one logical CPU fully busy; '
                          'all-cores percentage divides by logical CPU count. '
                          'Apple MPS/CUDA GPU work is not counted as CPU time.'),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
