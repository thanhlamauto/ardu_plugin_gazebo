#!/usr/bin/env python3
"""Pack saved Gazebo RGB frames and render Orin TensorRT depth as an MP4.

Inference is performed separately by tools/depth_tensorrt_probe.cpp in
--sequence mode. This script only prepares RGB bytes and draws its outputs.
"""

import argparse
import gzip
import json
import re
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib import colormaps


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "results/monocular_research/mentor_lidar_depth_benchmark_r3_20260930"
WORK = ROOT / "artifacts/orin_tensorrt_video"
WIDTH, HEIGHT = 640, 360


def font(size):
    candidates = (
        Path("/System/Library/Fonts/Supplemental/Arial Unicode.ttf"),
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    )
    for path in candidates:
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def pack(run, work):
    rows = [json.loads(line) for line in (run / "center_patch_depth_pairs.jsonl").open()]
    work.mkdir(parents=True, exist_ok=True)
    with (work / "sequence.rgb").open("wb") as target:
        for row in rows:
            path = run / "perception" / f"rgb_{row['frame']:06d}.png"
            with Image.open(path) as source:
                rgb = source.convert("RGB")
                if rgb.size != (WIDTH, HEIGHT):
                    raise ValueError(f"expected 640x360 RGB: {path}")
                target.write(rgb.tobytes())
    (work / "frames.json").write_text(json.dumps(rows, indent=2) + "\n")
    print(f"Packed {len(rows)} Gazebo frames: {work / 'sequence.rgb'}")


def render(run, work, output):
    rows = json.loads((work / "frames.json").read_text())
    depth_path = work / "depth_sequence.f32.gz"
    log_path = work / "benchmark.log"
    metrics = log_path.read_text() if log_path.is_file() else ""
    p50 = re.search(r"p50_ms ([0-9.]+)", metrics)
    fps = re.search(r"fps ([0-9.]+)", metrics)
    metric_line = (f"Orin C++ sequence: p50 {float(p50.group(1)):.1f} ms, "
                   f"{float(fps.group(1)):.1f} frames/s" if p50 and fps else
                   "Depth computed on Orin with TensorRT FP16")
    palette = (colormaps["turbo"](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    regular, small, title = font(22), font(18), font(29)
    output.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = subprocess.Popen([
        "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", "1320x520", "-r", "10", "-i", "-", "-an", "-c:v", "libx264",
        "-pix_fmt", "yuv420p", "-crf", "19", "-movflags", "+faststart", str(output),
    ], stdin=subprocess.PIPE)
    try:
        with gzip.open(depth_path, "rb") as depth_stream:
            for i, row in enumerate(rows):
                raw_depth = depth_stream.read(WIDTH * HEIGHT * 4)
                if len(raw_depth) != WIDTH * HEIGHT * 4:
                    raise ValueError(f"missing depth frame {i + 1}")
                depth = np.frombuffer(raw_depth, dtype="<f4").reshape((HEIGHT, WIDTH))
                with Image.open(run / "perception" / f"rgb_{row['frame']:06d}.png") as source:
                    rgb = source.convert("RGB")
                heat_index = np.clip(np.nan_to_num((10.0 - depth) / 8.0, nan=0) * 255,
                                     0, 255).astype(np.uint8)
                heat = Image.fromarray(palette[heat_index], "RGB")
                canvas = Image.new("RGB", (1320, 520), "#0b1321")
                draw = ImageDraw.Draw(canvas)
                draw.text((20, 14), "Gazebo camera RGB  ->  Orin TensorRT metric depth", font=title, fill="white")
                canvas.paste(rgb, (10, 66))
                canvas.paste(heat, (670, 66))
                draw.text((20, 435), f"frame {i + 1}/{len(rows)}  |  t={row['stamp_s']:.1f} s  |  RGB playback 10 FPS",
                          font=regular, fill="#e7edf4")
                center = float(np.median(depth[155:205, 295:345]))
                draw.text((685, 435), f"center depth {center:.2f} m  |  color 2-10 m", font=regular, fill="#e7edf4")
                draw.text((20, 478), metric_line + "  |  offline frames; no planner or flight",
                          font=small, fill="#a9b7c8")
                ffmpeg.stdin.write(canvas.tobytes())
            if depth_stream.read(1):
                raise ValueError("depth file has extra frames")
    finally:
        ffmpeg.stdin.close()
    if ffmpeg.wait() != 0:
        raise RuntimeError("ffmpeg failed")
    print(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("pack", "render"))
    parser.add_argument("--run", type=Path, default=RUN)
    parser.add_argument("--work", type=Path, default=WORK)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/orin_tensorrt_depth_video.mp4")
    args = parser.parse_args()
    if args.mode == "pack":
        pack(args.run, args.work)
    else:
        render(args.run, args.work, args.output)


if __name__ == "__main__":
    main()
