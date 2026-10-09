#!/usr/bin/env python3
"""Render a C++ probe's float32 depth map for an offline mentor demo.

This script only draws the result. Depth inference is performed by the C++
depth_anything_onnx_probe executable before this script is called.
"""

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib import colormaps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rgb", type=Path, help="640x360 RGB PNG")
    parser.add_argument("depth", type=Path, help="640x360 little-endian float32 depth")
    parser.add_argument("output", type=Path)
    parser.add_argument("--lidar-center-m", type=float, help="offline reference only")
    args = parser.parse_args()

    with Image.open(args.rgb) as source:
        rgb = source.convert("RGB")
    if rgb.size != (640, 360):
        parser.error("expected 640x360 RGB PNG")
    depth = np.fromfile(args.depth, dtype="<f4")
    if depth.size != 640 * 360:
        parser.error("expected 640x360 float32 depth")
    depth = depth.reshape((360, 640))
    center = float(np.median(depth[155:205, 295:345]))
    palette = (colormaps["turbo"](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    color_index = np.clip(np.nan_to_num((10 - depth) / 8, nan=0) * 255, 0, 255).astype(np.uint8)
    heat = Image.fromarray(palette[color_index], "RGB")

    canvas = Image.new("RGB", (1320, 540), "#0b1321")
    draw = ImageDraw.Draw(canvas)
    font_path = Path("/System/Library/Fonts/Supplemental/Arial Unicode.ttf")
    if font_path.exists():
        font = ImageFont.truetype(str(font_path), 24)
        small = ImageFont.truetype(str(font_path), 19)
    else:
        font = ImageFont.load_default()
        small = font
    canvas.paste(rgb, (10, 55))
    canvas.paste(heat, (670, 55))
    draw.text((10, 13), "RGB từ Gazebo", font=font, fill="white")
    draw.text((670, 13), "Depth C++ / Depth Anything V2 Metric Outdoor Small", font=font, fill="white")
    for x in (330, 990):
        draw.rectangle((x - 25, 210, x + 25, 260), outline="#ffbf69", width=3)
    draw.text((10, 435), f"Trung vị vùng giữa ảnh, C++: {center:.2f} m", font=font, fill="white")
    if args.lidar_center_m is not None:
        draw.text((670, 435), f"LiDAR tham chiếu offline: {args.lidar_center_m:.2f} m", font=font, fill="#50d5e5")
        draw.text((670, 471), f"Chênh lệch: {center - args.lidar_center_m:+.2f} m", font=small, fill="#ffbf69")
    draw.text((10, 471), "Màu depth cố định 2–10 m; gần hơn = màu nóng hơn", font=small, fill="#a9b7c8")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.output)
    print(f"{args.output}: center={center:.6f} m")


if __name__ == "__main__":
    main()
