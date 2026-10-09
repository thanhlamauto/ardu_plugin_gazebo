#!/usr/bin/env python3
"""Replay Gazebo RGB at camera rate and display measured depth throughput.

The C++ backend uses the deployment ONNX code. The optional MPS backend uses
PyTorch on the Mac GPU for a fast demonstration; it is not the C++ runtime.
LiDAR is read only for an offline comparison overlay.
"""

import argparse
from collections import deque
import json
import queue
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageTk
from matplotlib import colormaps


ROOT = Path(__file__).resolve().parents[1]
TRIAL = ROOT / "results/monocular_research/mentor_lidar_depth_benchmark_r3_20260930"
MODEL = ROOT / "results/monocular_research/depth_anything_v2_metric_outdoor_small_294x518_fixedpos.onnx"
STREAM = ROOT / "build/uav_navigation_ros_cpp_migration/depth_anything_onnx_stream"
MODEL_ID = "depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf"
REVISION = "2fd93bd764b15eea94dcf7763bba7ddc25007d0f"
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


def load_rgb(path):
    with Image.open(path) as source:
        rgb = source.convert("RGB")
    if rgb.size != (640, 360):
        raise RuntimeError(f"expected 640x360 RGB: {path}")
    return rgb


class CppPredictor:
    def __init__(self, args):
        self.process = subprocess.Popen(
            [str(args.stream), str(args.model), str(args.threads)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def predict(self, rgb):
        raw = rgb.tobytes()
        if len(raw) != FRAME_BYTES:
            raise RuntimeError("invalid RGB frame size")
        self.process.stdin.write(raw)
        self.process.stdin.flush()
        wall_ms, cpu_percent = struct.unpack("=dd", read_exact(self.process.stdout, 16))
        depth = np.frombuffer(read_exact(self.process.stdout, DEPTH_BYTES),
                              dtype="<f4").reshape((360, 640))
        return depth, wall_ms, cpu_percent

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()


class MpsPredictor:
    def __init__(self):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        if not torch.backends.mps.is_available():
            raise RuntimeError("Apple MPS không khả dụng trong Python hiện tại")
        torch.set_num_threads(1)
        self.torch = torch
        self.processor = AutoImageProcessor.from_pretrained(
            MODEL_ID, revision=REVISION, local_files_only=True)
        self.model = AutoModelForDepthEstimation.from_pretrained(
            MODEL_ID, revision=REVISION, local_files_only=True).to("mps").eval()

    def predict(self, rgb):
        start = time.monotonic()
        cpu_start = time.process_time()
        inputs = self.processor(images=rgb, return_tensors="pt",
                                size={"height": 518, "width": 518}).to("mps")
        with self.torch.inference_mode():
            predicted = self.model(**inputs).predicted_depth
            depth = self.torch.nn.functional.interpolate(
                predicted[:, None], size=(360, 640), mode="bicubic",
                align_corners=False)[0, 0].cpu().numpy()
        wall_s = time.monotonic() - start
        return depth, wall_s * 1000, 100 * (time.process_time() - cpu_start) / wall_s

    def close(self):
        pass


def run_inference(args, selected, pending, output, stop):
    predictor = None
    try:
        predictor = MpsPredictor() if args.backend == "mps" else CppPredictor(args)
        predictor.predict(load_rgb(selected[0][0]))  # warm up before starting video
        output.put(("ready",))
        if args.headless:
            tasks = ((load_rgb(path), pair, time.monotonic()) for path, pair in selected)
        else:
            def live_tasks():
                while not stop.is_set():
                    try:
                        yield pending.get(timeout=0.1)
                    except queue.Empty:
                        continue
            tasks = live_tasks()
        for rgb, pair, captured_at in tasks:
            if stop.is_set():
                break
            depth, wall_ms, cpu_percent = predictor.predict(rgb)
            completed_at = time.monotonic()
            center = float(np.median(depth[155:205, 295:345]))
            index = np.clip(np.nan_to_num((10 - depth) / 8, nan=0) * 255,
                            0, 255).astype(np.uint8)
            heat = Image.fromarray(PALETTE[index], "RGB")
            output.put(("depth", heat, pair, center, wall_ms, cpu_percent,
                        (completed_at - captured_at) * 1000, completed_at))
            if not args.headless:
                print(f"depth frame {pair['frame']}: {wall_ms:.0f} ms "
                      f"backend={args.backend}", flush=True)
    except Exception as error:
        if isinstance(predictor, CppPredictor) and predictor.process.poll() is not None:
            detail = predictor.process.stderr.read().decode(errors="replace")
            error = RuntimeError(f"{error}; C++: {detail}")
        print(f"Demo depth lỗi: {error}", file=sys.stderr, flush=True)
        output.put(error)
    finally:
        if predictor is not None:
            predictor.close()
        output.put(None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("cpp", "mps"), default="cpp")
    parser.add_argument("--model", type=Path, default=MODEL)
    parser.add_argument("--stream", type=Path, default=STREAM)
    parser.add_argument("--trial", type=Path, default=TRIAL)
    parser.add_argument("--frames", type=int, default=131)
    parser.add_argument("--camera-hz", type=float, default=10.0)
    parser.add_argument("--threads", type=int, default=4,
                        help="OpenCV CPU threads for the C++ backend (default 4)")
    parser.add_argument("--headless", action="store_true", help="validate one pass without GUI")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    if args.camera_hz <= 0:
        parser.error("--camera-hz must be positive")
    if not 1 <= args.threads <= 8:
        parser.error("--threads must be 1..8")
    if args.backend == "cpp" and not args.model.is_file():
        parser.error(f"ONNX model missing: {args.model}")
    if args.backend == "cpp" and not args.stream.is_file():
        parser.error(f"C++ stream executable missing: {args.stream}; build target depth_anything_onnx_stream")
    selected = frames(args.trial, args.frames)
    for path, _ in selected:
        if not path.is_file():
            parser.error(f"RGB frame missing: {path}")

    output = queue.Queue()
    pending = queue.Queue(maxsize=1)
    stop = threading.Event()
    worker = threading.Thread(target=run_inference,
                              args=(args, selected, pending, output, stop), daemon=True)
    worker.start()

    if args.headless:
        while True:
            item = output.get()
            if item is None:
                break
            if isinstance(item, Exception):
                raise item
            if item[0] == "ready":
                print(f"backend={args.backend} ready", flush=True)
                continue
            _, _, pair, center, wall_ms, cpu, _, _ = item
            print(f"frame={pair['frame']} depth={center:.2f}m LiDAR={pair['lidar_depth_m']:.2f}m "
                  f"inference={wall_ms:.0f}ms CPU={cpu:.0f}% one core", flush=True)
        worker.join(timeout=3)
        return

    import tkinter as tk
    root = tk.Tk()
    backend_name = "Apple MPS / PyTorch" if args.backend == "mps" else "C++ / OpenCV CPU"
    root.title(f"UAV monocular depth — {backend_name}")
    root.configure(bg="#0b1321")
    root.attributes("-topmost", True)
    root.lift()
    root.after(1500, lambda: root.attributes("-topmost", False))
    print(f"Đã mở cửa sổ demo {backend_name}; nhấn Space để tạm dừng.", flush=True)
    title = tk.Label(root, text=f"Video RGB 10 Hz → Depth Anything V2 Metric Outdoor Small ({backend_name})",
                     font=("Arial", 18), fg="white", bg="#0b1321")
    title.pack(pady=12)
    panels = tk.Frame(root, bg="#0b1321")
    panels.pack()
    left = tk.Label(panels, bg="#0b1321")
    left.grid(row=0, column=0, padx=8)
    right = tk.Label(panels, bg="#0b1321")
    right.grid(row=0, column=1, padx=8)
    rgb_label = tk.Label(panels, text="RGB Gazebo • đang chờ model", font=("Arial", 13),
                         fg="white", bg="#0b1321")
    rgb_label.grid(row=1, column=0, pady=5)
    depth_label = tk.Label(panels, text=f"Depth {backend_name} • đang chờ model",
                           font=("Arial", 13), fg="white", bg="#0b1321")
    depth_label.grid(row=1, column=1, pady=5)
    metric = tk.Label(root, text=f"Đang tải model {backend_name}…", font=("Arial", 16),
                      fg="#50d5e5", bg="#0b1321")
    metric.pack(pady=14)
    note = tk.Label(root, text="LiDAR chỉ đối chiếu offline • Phát lại video Gazebo, không bay • FPS Mac ≠ Jetson",
                    font=("Arial", 13), fg="#a9b7c8", bg="#0b1321")
    note.pack(pady=5)

    paused = False
    ready = False
    camera_index = 0
    camera_count = 0
    dropped_count = 0
    current_rgb_frame = None
    camera_period_s = 1.0 / args.camera_hz
    next_camera_at = time.monotonic() + camera_period_s
    camera_times = deque(maxlen=20)
    depth_times = deque(maxlen=20)

    def fps(times):
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0

    def toggle_pause():
        nonlocal paused
        paused = not paused
        button.configure(text="Tiếp tục" if paused else "Tạm dừng")

    button = tk.Button(root, text="Tạm dừng", command=toggle_pause)
    button.pack(pady=10)
    root.bind("<space>", lambda _event: toggle_pause())

    def play_frame():
        nonlocal camera_index, camera_count, dropped_count, current_rgb_frame, next_camera_at
        if ready and not paused:
            path, pair = selected[camera_index]
            camera_index = (camera_index + 1) % len(selected)
            try:
                rgb = load_rgb(path)
            except Exception as error:
                metric.configure(text=f"Lỗi đọc video: {error}", fg="#ff7676")
                return
            captured_at = time.monotonic()
            camera_times.append(captured_at)
            camera_count += 1
            current_rgb_frame = pair["frame"]
            left.photo = ImageTk.PhotoImage(rgb.resize((560, 315)))
            left.configure(image=left.photo)
            rgb_label.configure(text=f"RGB Gazebo • {fps(camera_times):.1f} FPS • frame {current_rgb_frame}")
            if pending.full():
                pending.get_nowait()
                dropped_count += 1
            pending.put_nowait((rgb, pair, captured_at))
            if camera_count % 10 == 0:
                print(f"RGB {fps(camera_times):.1f} FPS | depth {fps(depth_times):.1f} FPS | "
                      f"bỏ qua {dropped_count} frame | backend={args.backend}", flush=True)
            next_camera_at += camera_period_s
            if next_camera_at < time.monotonic():
                next_camera_at = time.monotonic() + camera_period_s
        else:
            next_camera_at = time.monotonic() + camera_period_s
        root.after(max(1, round((next_camera_at - time.monotonic()) * 1000)), play_frame)

    def pump():
        nonlocal ready
        try:
            while True:
                item = output.get_nowait()
                if item is None:
                    return
                if isinstance(item, Exception):
                    metric.configure(text=f"Lỗi: {item}", fg="#ff7676")
                    return
                if item[0] == "ready":
                    ready = True
                    metric.configure(text="Model đã sẵn sàng. Đang phát RGB và đo FPS depth…")
                    continue
                _, heat, pair, center, wall_ms, cpu, age_ms, completed_at = item
                depth_times.append(completed_at)
                right.photo = ImageTk.PhotoImage(heat.resize((560, 315)))
                right.configure(image=right.photo)
                depth_label.configure(text=f"Depth {backend_name} • {fps(depth_times):.1f} FPS • frame {pair['frame']}")
                lidar = pair["lidar_depth_m"]
                metric.configure(text=f"Depth {center:.2f} m  |  LiDAR {lidar:.2f} m  |  "
                                      f"Sai lệch {center-lidar:+.2f} m  |  "
                                      f"RGB bỏ qua: {dropped_count}/{camera_count}\n"
                                      f"Suy luận {wall_ms:.0f} ms  |  Tuổi kết quả {age_ms:.0f} ms  |  "
                                      f"CPU host {cpu:.0f}%")
        except queue.Empty:
            pass
        root.after(50, pump)

    def close():
        stop.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    root.after(50, pump)
    root.after(round(1000 / args.camera_hz), play_frame)
    root.mainloop()
    stop.set()
    worker.join(timeout=3)


if __name__ == "__main__":
    main()
