#!/usr/bin/env python3
"""Render a short, evidence-labeled video from the saved mentor benchmark run."""

import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib import colormaps


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN = ROOT / "results/monocular_research/mentor_lidar_depth_benchmark_r3_20260930"
FONT = Path("/System/Library/Fonts/Supplemental/Arial Unicode.ttf")


def rows(path):
    return [json.loads(line) for line in path.open()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/depth_demo.mp4")
    args = parser.parse_args()
    run = args.run
    pairs = rows(run / "center_patch_depth_pairs.jsonl")
    tracks = {r["frame"]: r for r in rows(run / "multiframe_ekf_frames.jsonl")}
    result = json.loads((run / "result.json").read_text())
    analysis = json.loads((run / "lidar_depth_analysis.json").read_text())
    phases = result["benchmark_phases"]
    start_move = phases["move_forward"]["start_sim_s"]
    end_move = phases["move_forward"]["end_sim_s"]
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    regular = {size: ImageFont.truetype(str(FONT), size) for size in (17, 20, 23, 27, 30, 36, 48)}
    palette = (colormaps["turbo"](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    bg = "#0b1321"
    white = "#f5f7fa"
    muted = "#a9b7c8"
    cyan = "#50d5e5"
    amber = "#ffbf69"
    red = "#ff7676"
    fps = 10
    w, h = 1280, 720
    panel_y = 117
    panel_size = (600, 338)
    left_x, right_x = 30, 650

    def label(draw, xy, value, size=20, color=white):
        draw.text(xy, value, font=regular[size], fill=color)

    def base(title, subtitle):
        im = Image.new("RGB", (w, h), bg)
        d = ImageDraw.Draw(im)
        label(d, (30, 22), title, 36)
        label(d, (31, 69), subtitle, 20, muted)
        d.line((30, 99, 1250, 99), fill="#2b3b4e", width=2)
        return im, d

    def center_text(draw, y, value, size=27, color=white):
        box = draw.textbbox((0, 0), value, font=regular[size])
        label(draw, ((w - (box[2] - box[0])) // 2, y), value, size, color)

    def card(lines, foot):
        im, d = base("UAV monocular depth | Gazebo SITL", "Benchmark 30/09/2026  •  RGB 640×360, 10 Hz  •  tiến thẳng 0,5 m/s")
        d.rounded_rectangle((100, 157, 1180, 553), radius=18, fill="#142235", outline="#30465e", width=2)
        for idx, (head, body) in enumerate(lines):
            y = 187 + idx * 94
            center_text(d, y, head, 30, cyan if idx == 0 else white)
            center_text(d, y + 40, body, 23, muted)
        center_text(d, 608, foot, 20, amber)
        return im

    intro = card([
        ("Camera RGB → dự đoán khoảng cách", "Depth Anything V2 Metric Outdoor Small"),
        ("LiDAR 3D → số đo tham chiếu", "Chỉ dùng để đánh giá sau chuyến bay"),
        ("Bay thử: hover → tiến → hover", "Pose bay lấy từ EKF ArduPilot"),
    ], "Video ghép từ ảnh và log của cùng một lượt benchmark")
    outro = card([
        ("Kết quả của lượt chạy này", f"MAE RGB/LiDAR: {analysis['all_visible_returns']['mae_m']:.2f} m  •  p95 nhận ảnh → publish: {analysis['receive_to_publish_ms']['p95']:.0f} ms"),
        ("Nhánh landmark nhiều ảnh", "MAE 0,115 m trên landmark được chọn; có depth sau ~1,44 m di chuyển"),
        ("Chưa đủ để tránh vật cản tự động", "Cần kiểm tra thêm độ phủ, precision/recall và độ trễ end-to-end"),
    ], "LiDAR không đi vào predictor; không chạy planner hoặc OpenVINS")

    proc = subprocess.Popen([
        "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo",
        "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
        "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "19", "-preset", "medium",
        "-movflags", "+faststart", str(output),
    ], stdin=subprocess.PIPE)

    def push(im, count=1):
        frame = im.tobytes()
        for _ in range(count):
            proc.stdin.write(frame)

    try:
        push(intro, 20)
        for n, pair in enumerate(pairs):
            frame_id = pair["frame"]
            rgb = Image.open(run / "perception" / f"rgb_{frame_id:06d}.png").convert("RGB")
            depth = np.load(run / "perception" / f"depth_{frame_id:06d}.npy")
            # Fixed 2–10 m scale throughout the film; clipped values are labeled.
            color_index = np.clip(np.nan_to_num((10 - depth) / 8, nan=0) * 255, 0, 255).astype(np.uint8)
            heat = Image.fromarray(palette[color_index], "RGB")
            im, d = base("Camera RGB và depth dự đoán", "Cùng timestamp • LiDAR chỉ xuất hiện trong phần đối chiếu offline")
            im.paste(rgb.resize(panel_size, Image.Resampling.LANCZOS), (left_x, panel_y))
            im.paste(heat.resize(panel_size, Image.Resampling.LANCZOS), (right_x, panel_y))
            for x in (left_x, right_x):
                d.rectangle((x, panel_y, x + 599, panel_y + 337), outline="#68839d", width=2)
            label(d, (left_x + 12, panel_y + 10), "CAMERA RGB", 23)
            label(d, (right_x + 12, panel_y + 10), "METRIC DEPTH  |  2–10 m", 23)
            for x in (left_x + 300, right_x + 300):
                d.rectangle((x - 23, panel_y + 169 - 23, x + 23, panel_y + 169 + 23), outline=amber, width=3)
            stamp = pair["stamp_s"]
            phase = "HOVER TRƯỚC" if stamp < start_move else "TIẾN THẲNG 0,5 m/s" if stamp <= end_move else "HOVER SAU"
            d.rounded_rectangle((30, 475, 1250, 652), radius=14, fill="#16263a")
            label(d, (51, 490), f"t = {stamp:.1f} s", 23, muted)
            label(d, (265, 490), phase, 23, cyan)
            label(d, (675, 490), f"Landmark depth: {tracks[frame_id]['confident_landmarks']}", 23, muted)
            label(d, (51, 533), f"RGB: {pair['monocular_depth_m']:.2f} m", 30, white)
            label(d, (470, 533), f"LiDAR: {pair['lidar_depth_m']:.2f} m", 30, cyan)
            error = pair["monocular_depth_m"] - pair["lidar_depth_m"]
            label(d, (899, 533), f"Sai lệch: {error:+.2f} m", 27, red if abs(error) > 1 else amber)
            label(d, (51, 592), "Trung vị vùng 50×50 px giữa ảnh • LiDAR được chiếu lên RGB sau chuyến bay", 20, muted)
            d.rounded_rectangle((30, 675, 1250, 684), radius=4, fill="#30465e")
            d.rounded_rectangle((30, 675, 30 + int(1220 * (n + 1) / len(pairs)), 684), radius=4, fill=cyan)
            if n == 75:
                im.save(output.with_suffix(".png"))
            push(im)
        push(outro, 40)
    finally:
        proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError("ffmpeg failed")
    print(output)


if __name__ == "__main__":
    main()
