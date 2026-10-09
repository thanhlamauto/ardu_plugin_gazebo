#!/usr/bin/env python3
"""Live Mac display for recorded RGB -> persistent C++ ONNX depth inference.

Python only replays images, draws the result and overlays offline LiDAR labels.
The depth predictor is the C++ depth_anything_onnx_stream executable.
"""

import argparse
import json
import queue
import struct
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
from PIL import Image, ImageTk
from matplotlib import colormaps


ROOT = Path(__file__).resolve().parents[1]
TRIAL = ROOT / "results/monocular_research/mentor_lidar_depth_benchmark_r3_20260930"
MODEL = ROOT / "results/monocular_research/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx"
STREAM = ROOT / "build/uav_navigation_ros_cpp_migration/depth_anything_onnx_stream"
FRAME_BYTES = 640 * 360 * 3
DEPTH_BYTES = 640 * 360 * 4
PALETTE = (colormaps["turbo"](np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)


def read_exact(pipe, length):
    parts = []
    received = 0
    while received < length:
        part = pipe.read(length - received)
        if not part:
            raise RuntimeError("C++ stream stopped before sending a full depth frame")
        parts.append(part)
        received += len(part)
    return b"".join(parts)


def frames(trial, count):
    pairs_file = trial / "center_patch_depth_pairs.jsonl"
    if pairs_file.is_file():
        pairs = [json.loads(line) for line in pairs_file.open()]
        indexes = np.linspace(0, len(pairs) - 1, min(count, len(pairs)), dtype=int)
        return [(trial / "perception" / f"rgb_{pairs[i]['frame']:06d}.png", pairs[i])
                for i in indexes]
    sample = ROOT / "docs/mentor_depth_evidence/cpp/sample_rgb_000220.png"
    return [(sample, {"frame": 220, "stamp_s": 30.0,
                      "lidar_depth_m": 5.061122417449951})]


def run_inference(args, selected, output, stop, paused):
    process = subprocess.Popen([str(args.stream), str(args.model), str(args.threads)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    try:
        while not stop.is_set():
            for rgb_file, pair in selected:
                if stop.is_set():
                    break
                while paused.is_set() and not stop.wait(0.1):
                    pass
                if stop.is_set():
                    break
                with Image.open(rgb_file) as source:
                    rgb = source.convert("RGB")
                if rgb.size != (640, 360):
                    raise RuntimeError(f"expected 640x360 RGB: {rgb_file}")
                raw = rgb.tobytes()
                if len(raw) != FRAME_BYTES:
                    raise RuntimeError("invalid RGB frame size")
                process.stdin.write(raw)
                process.stdin.flush()
                wall_ms, cpu_percent = struct.unpack("=dd", read_exact(process.stdout, 16))
                depth = np.frombuffer(read_exact(process.stdout, DEPTH_BYTES),
                                      dtype="<f4").reshape((360, 640))
                center = float(np.median(depth[155:205, 295:345]))
                index = np.clip(np.nan_to_num((10 - depth) / 8, nan=0) * 255,
                                0, 255).astype(np.uint8)
                heat = Image.fromarray(PALETTE[index], "RGB")
                output.put((rgb, heat, pair, center, wall_ms, cpu_percent))
                if not args.headless:
                    print(f"frame {pair['frame']}: C++ {center:.2f} m, "
                          f"{wall_ms:.0f} ms, CPU {cpu_percent:.0f}% một lõi", flush=True)
            if args.headless:
                break
    except Exception as error:
        if process.poll() is not None:
            error = RuntimeError(f"{error}; C++: {process.stderr.read().decode(errors='replace')}")
        print(f"Demo C++ lỗi: {error}", file=sys.stderr, flush=True)
        output.put(error)
    finally:
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        output.put(None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=MODEL)
    parser.add_argument("--stream", type=Path, default=STREAM)
    parser.add_argument("--trial", type=Path, default=TRIAL)
    parser.add_argument("--frames", type=int, default=16)
    parser.add_argument("--threads", type=int, default=4,
                        help="OpenCV CPU threads (default 4); use 1 for reference measurements")
    parser.add_argument("--headless", action="store_true", help="validate one pass without GUI")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    if not 1 <= args.threads <= 8:
        parser.error("--threads must be 1..8")
    if not args.model.is_file():
        parser.error(f"ONNX model missing: {args.model}")
    if not args.stream.is_file():
        parser.error(f"C++ stream executable missing: {args.stream}; build target depth_anything_onnx_stream")
    selected = frames(args.trial, args.frames)
    for path, _ in selected:
        if not path.is_file():
            parser.error(f"RGB frame missing: {path}")

    output = queue.Queue()
    stop = threading.Event()
    paused = threading.Event()
    worker = threading.Thread(target=run_inference,
                              args=(args, selected, output, stop, paused), daemon=True)
    worker.start()

    if args.headless:
        while True:
            item = output.get()
            if item is None:
                break
            if isinstance(item, Exception):
                raise item
            _, _, pair, center, wall_ms, cpu = item
            print(f"frame={pair['frame']} C++={center:.2f}m LiDAR={pair['lidar_depth_m']:.2f}m "
                  f"inference={wall_ms:.0f}ms CPU={cpu:.0f}% one core", flush=True)
        worker.join(timeout=3)
        return

    import tkinter as tk
    root = tk.Tk()
    root.title("UAV monocular C++ depth — demo trực tiếp trên Mac")
    root.configure(bg="#0b1321")
    root.attributes("-topmost", True)
    root.lift()
    root.after(1500, lambda: root.attributes("-topmost", False))
    print(f"Đã mở cửa sổ 'UAV monocular C++ depth' ({args.threads} luồng CPU); "
          "nhấn Space để tạm dừng.", flush=True)
    title = tk.Label(root, text="Ảnh Gazebo đã ghi → Depth Anything V2 Metric Outdoor Small (C++)",
                     font=("Arial", 18), fg="white", bg="#0b1321")
    title.pack(pady=12)
    panels = tk.Frame(root, bg="#0b1321")
    panels.pack()
    left = tk.Label(panels, bg="#0b1321")
    left.grid(row=0, column=0, padx=8)
    right = tk.Label(panels, bg="#0b1321")
    right.grid(row=0, column=1, padx=8)
    tk.Label(panels, text="RGB từ Gazebo (đã ghi)", font=("Arial", 13),
             fg="white", bg="#0b1321").grid(row=1, column=0, pady=5)
    tk.Label(panels, text="Depth C++ (2–10 m)", font=("Arial", 13),
             fg="white", bg="#0b1321").grid(row=1, column=1, pady=5)
    metric = tk.Label(root, text="Đang tải model C++…", font=("Arial", 16),
                      fg="#50d5e5", bg="#0b1321")
    metric.pack(pady=14)
    note = tk.Label(root, text="LiDAR chỉ đối chiếu offline • CPU Mac, chưa đo Jetson • Phát lại ảnh, không bay Gazebo",
                    font=("Arial", 13), fg="#a9b7c8", bg="#0b1321")
    note.pack(pady=5)

    def toggle_pause():
        if paused.is_set():
            paused.clear()
            button.configure(text="Tạm dừng")
        else:
            paused.set()
            button.configure(text="Tiếp tục")

    button = tk.Button(root, text="Tạm dừng", command=toggle_pause)
    button.pack(pady=10)
    root.bind("<space>", lambda _event: toggle_pause())

    def pump():
        try:
            while True:
                item = output.get_nowait()
                if item is None:
                    return
                if isinstance(item, Exception):
                    metric.configure(text=f"Lỗi: {item}", fg="#ff7676")
                    return
                rgb, heat, pair, center, wall_ms, cpu = item
                left.photo = ImageTk.PhotoImage(rgb.resize((560, 315)))
                right.photo = ImageTk.PhotoImage(heat.resize((560, 315)))
                left.configure(image=left.photo)
                right.configure(image=right.photo)
                lidar = pair["lidar_depth_m"]
                metric.configure(text=f"Frame {pair['frame']}  |  C++ {center:.2f} m  |  "
                                      f"LiDAR {lidar:.2f} m  |  Sai lệch {center-lidar:+.2f} m\n"
                                      f"Suy luận {wall_ms:.0f} ms  |  CPU {cpu:.0f}% "
                                      f"(≈{cpu/100:.1f} lõi)")
        except queue.Empty:
            pass
        root.after(50, pump)

    def close():
        stop.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    root.after(50, pump)
    root.mainloop()
    stop.set()
    worker.join(timeout=3)


if __name__ == "__main__":
    main()
