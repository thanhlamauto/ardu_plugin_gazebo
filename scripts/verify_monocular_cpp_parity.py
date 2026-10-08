#!/usr/bin/env python3
"""Offline comparison of the C++ ONNX depth probe with saved Python depth.

Python is only used here for test orchestration; the probe and flight runtime
are C++ executables. LiDAR/pose are not inputs to either predictor.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, nargs='+', default=[0, 120, 240])
    args = parser.parse_args()
    rows = []
    with tempfile.TemporaryDirectory() as temporary:
        raw = Path(temporary) / 'rgb.raw'
        prediction = Path(temporary) / 'depth.float32'
        for index in args.frames:
            frame = args.trial / 'perception' / f'rgb_{index:06d}.png'
            reference = args.trial / 'perception' / f'depth_{index:06d}.npy'
            with Image.open(frame) as image:
                raw.write_bytes(image.convert('RGB').tobytes())
            completed = subprocess.run(
                [str(args.probe.resolve()), str(args.model.resolve()),
                 str(raw), str(prediction)], text=True, capture_output=True,
                check=True)
            expected = np.load(reference)
            actual = np.fromfile(prediction, dtype='<f4').reshape(expected.shape)
            difference = np.abs(actual - expected)
            rows.append(dict(frame=index, mae_m=float(difference.mean()),
                             p95_absolute_error_m=float(np.percentile(difference, 95)),
                             maximum_absolute_error_m=float(difference.max()),
                             center_median_difference_m=float(
                                 np.median(actual[155:205, 295:345]) -
                                 np.median(expected[155:205, 295:345])),
                             probe_stdout=completed.stdout.strip()))
    output = dict(model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                  comparison='C++ OpenCV ONNX output versus saved Python MPS depth',
                  trial=args.trial.name, frames=rows,
                  mean_frame_mae_m=float(np.mean([row['mae_m'] for row in rows])),
                  maximum_pixel_error_m=max(row['maximum_absolute_error_m'] for row in rows))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
