#!/usr/bin/env python3
"""Offline-only export of the pinned metric-depth checkpoint for C++ runtime.

Run this on a development machine. Python/PyTorch are not needed on the UAV.
The ONNX graph has fixed 1x3x294x518 input for the 640x360 camera.
"""

import argparse
import hashlib
import json
from pathlib import Path
import types


MODEL = 'depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf'
REVISION = '2fd93bd764b15eea94dcf7763bba7ddc25007d0f'
INPUT_HEIGHT = 294
INPUT_WIDTH = 518


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    import onnx
    import torch
    from transformers import AutoModelForDepthEstimation

    torch.set_num_threads(1)
    model = AutoModelForDepthEstimation.from_pretrained(
        MODEL, revision=REVISION).eval()
    embeddings = model.backbone.embeddings
    patches = (INPUT_HEIGHT // embeddings.patch_size) * (
        INPUT_WIDTH // embeddings.patch_size)
    dummy_tokens = torch.zeros(1, patches + 1,
                               embeddings.position_embeddings.shape[-1])
    with torch.inference_mode():
        fixed_position = embeddings.interpolate_pos_encoding(
            dummy_tokens, INPUT_HEIGHT, INPUT_WIDTH).detach()
    # Fixed camera resolution lets the exact bicubic positional interpolation
    # become a constant. OpenCV DNN cannot import that cubic ONNX Resize op.
    embeddings.register_buffer('fixed_position_embeddings', fixed_position)

    def fixed_pos(self, tokens, height, width):
        return self.fixed_position_embeddings

    embeddings.interpolate_pos_encoding = types.MethodType(fixed_pos, embeddings)

    class PredictionOnly(torch.nn.Module):
        def __init__(self, base):
            super().__init__()
            self.base = base

        def forward(self, pixel_values):
            return self.base(pixel_values=pixel_values).predicted_depth

    args.output.parent.mkdir(parents=True, exist_ok=True)
    example = torch.zeros(1, 3, INPUT_HEIGHT, INPUT_WIDTH)
    torch.onnx.export(PredictionOnly(model).eval(), example, str(args.output),
                      input_names=['pixel_values'],
                      output_names=['predicted_depth'], opset_version=17,
                      dynamo=False, do_constant_folding=True)
    onnx.checker.check_model(str(args.output))
    data = args.output.read_bytes()
    metadata = dict(model=MODEL, revision=REVISION, opset=17,
                    input_shape=[1, 3, INPUT_HEIGHT, INPUT_WIDTH],
                    camera_shape=[640, 360], output_shape=[1, INPUT_HEIGHT, INPUT_WIDTH],
                    positional_interpolation='precomputed bicubic for fixed camera resolution',
                    sha256=hashlib.sha256(data).hexdigest(), size_bytes=len(data),
                    torch=torch.__version__)
    args.output.with_suffix('.onnx.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
