#!/usr/bin/env python3
"""Record evaluation-only Gazebo LiDAR scans; never publishes perception data."""
import argparse
import json
import signal
import threading
import time
from pathlib import Path

import numpy as np


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec*1e-9


def decode_cloud(msg):
    fields = {field.name: field for field in msg.field}
    if any(name not in fields for name in ('x','y','z')):
        raise ValueError('LiDAR cloud must have x/y/z fields')
    width, height, step = int(msg.width), int(msg.height), int(msg.point_step)
    row_step = int(msg.row_step) or width*step
    if width < 1 or height < 1 or step < 1 or row_step < width*step:
        raise ValueError('invalid LiDAR cloud shape')
    if len(msg.data) < height*row_step:
        raise ValueError('truncated LiDAR cloud')
    endian = '>' if msg.is_bigendian else '<'
    axes = []
    for name in ('x','y','z'):
        field = fields[name]
        if field.datatype not in (6,7):
            raise ValueError(f'unsupported {name} datatype')
        itemsize = 4 if field.datatype == 6 else 8
        if field.offset < 0 or field.offset+itemsize > step:
            raise ValueError(f'invalid {name} field offset')
        axis = np.ndarray((height,width),dtype=endian+f'f{itemsize}',
                          buffer=msg.data,offset=field.offset,strides=(row_step,step))
        axes.append(axis.reshape(-1))
    xyz = np.column_stack(axes).astype('<f4')
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--timeout',type=float,default=180.)
    parser.add_argument('--topic',default='/benchmark/lidar_ground_truth/points')
    args = parser.parse_args()
    from gz.transport13 import Node
    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
    args.output_dir.mkdir(parents=True,exist_ok=True)
    node = Node()
    lock = threading.Lock()
    queue = []
    received = [0]
    def callback(msg):
        with lock:
            received[0] += 1
            queue.append(msg)
            if len(queue)>10:
                queue.pop(0)
    if not node.subscribe(PointCloudPacked,args.topic,callback):
        raise RuntimeError(f'LiDAR subscription failed: {args.topic}')
    stop = threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    start = time.monotonic()
    count = 0
    with (args.output_dir/'scans.jsonl').open('w') as index:
        while not stop.is_set() and time.monotonic()-start<args.timeout:
            with lock:
                msg = queue.pop(0) if queue else None
            if msg is None:
                time.sleep(.005)
                continue
            points = decode_cloud(msg)
            name = f'scan_{count:06d}.npz'
            np.savez_compressed(args.output_dir/name,xyz=points)
            index.write(json.dumps(dict(stamp_s=stamp(msg),file=name,
                                        points=len(points),frame_id='lidar_ground_truth'))+'\n')
            index.flush()
            count += 1
    (args.output_dir/'summary.json').write_text(json.dumps(dict(
        received=received[0],saved=count,dropped=max(0,received[0]-count),
        topic=args.topic,role='evaluation only'),indent=2)+'\n')


if __name__ == '__main__':
    main()
