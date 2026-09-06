
import argparse
import math
import os
import struct
import threading
import time
from typing import Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float, float]

LIDAR_MAX_M = 30.0
LIDAR_MIN_M = 0.15
DEFAULT_SEND_HZ = 5.0
DEFAULT_MAX_POINTS = 180

# The model places sensor_suite_link at x=0.08 m, z=+0.16 m relative to the
# vehicle base_link. Coordinates from the Gazebo LiDAR are FLU and are
# expressed from the sensor origin. The default is therefore BODY_FRD.
DEFAULT_SENSOR_OFFSET_BODY_FRD: Point = (0.08, 0.0, -0.16)


def load_mavutil():
    """Load pymavlink with the ArduPilot dialect forced to MAVLink 2.

    pymavlink selects v1.0 at import time unless MAVLINK20 is already in the
    environment. OBSTACLE_DISTANCE_3D has message id 11037, so MAVLink 1
    cannot carry it.
    """

    os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")
    os.environ["MAVLINK20"] = "1"

    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise SystemExit(
            "[error] cần pymavlink. Hãy chạy bridge bằng Python của env "
            "ardupilot-rviz hoặc cài pymavlink."
        ) from exc

    # This also fixes callers that imported pymavlink before this script.
    mavutil.set_dialect("ardupilotmega")
    wire_version = float(mavutil.mavlink.WIRE_PROTOCOL_VERSION)
    if wire_version < 2.0 or not hasattr(
        mavutil.mavlink, "MAVLink_obstacle_distance_3d_message"
    ):
        raise SystemExit(
            "[error] pymavlink hiện không có dialect MAVLink 2 / "
            "OBSTACLE_DISTANCE_3D. Hãy đặt MAVLINK20=1 và dùng đúng env."
        )
    return mavutil


def open_mavlink(mav_target: str):
    """Open MAVLink 2 and wait for the ArduPilot heartbeat."""

    mavutil = load_mavutil()
    mav = mavutil.mavlink_connection(
        mav_target,
        source_system=255,
        source_component=191,
        dialect="ardupilotmega",
    )
    print(f"[mavlink] connecting to {mav_target}, waiting for heartbeat...")
    heartbeat = mav.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise SystemExit(f"[error] không nhận được heartbeat từ {mav_target}")

    print(
        f"[mavlink] connected to system {heartbeat.get_srcSystem():d}:"
        f"{heartbeat.get_srcComponent():d} with MAVLink {mav.WIRE_PROTOCOL_VERSION}"
    )
    return mav, mavutil


def sensor_flu_to_body_frd(point_flu: Point, sensor_offset_body_frd: Point) -> Point:
    """Transform a point from the Gazebo sensor FLU frame to BODY_FRD."""

    x_flu, y_flu, z_flu = point_flu
    offset_x, offset_y, offset_z = sensor_offset_body_frd
    # Same orientation in the SDF: FLU -> FRD means negate left/up axes.
    return offset_x + x_flu, offset_y - y_flu, offset_z - z_flu


def _angle_bin(angle_deg: float, width_deg: float, period_deg: float) -> int:
    return int(math.floor((angle_deg % period_deg) / width_deg))


def point_cloud_to_obstacles(
    points_flu: Iterable[Point],
    *,
    max_points: int = DEFAULT_MAX_POINTS,
    max_range: float = LIDAR_MAX_M,
    min_range: float = LIDAR_MIN_M,
    sensor_offset_body_frd: Point = DEFAULT_SENSOR_OFFSET_BODY_FRD,
    yaw_bin_deg: float = 5.0,
    pitch_bin_deg: float = 5.0,
    z_min: Optional[float] = None,
    z_max: Optional[float] = None,
) -> List[Point]:
    """Filter, transform and angularly downsample a FLU point cloud.

    One nearest point is retained per yaw/pitch cell. Sorting by distance
    before applying max_points preserves the closest geometry when the
    640 x 16 cloud contains more cells than the MAVLink budget.
    """

    if max_points < 1:
        return []
    if yaw_bin_deg <= 0.0 or pitch_bin_deg <= 0.0:
        raise ValueError("angular bin width must be positive")

    nearest_by_cell = {}
    for point in points_flu:
        if len(point) < 3:
            continue
        x_flu, y_flu, z_flu = (float(point[0]), float(point[1]), float(point[2]))
        if not all(math.isfinite(value) for value in (x_flu, y_flu, z_flu)):
            continue
        sensor_range = math.sqrt(x_flu * x_flu + y_flu * y_flu + z_flu * z_flu)
        if sensor_range < min_range or sensor_range > max_range:
            continue
        if z_min is not None and z_flu < z_min:
            continue
        if z_max is not None and z_flu > z_max:
            continue

        point_frd = sensor_flu_to_body_frd(
            (x_flu, y_flu, z_flu), sensor_offset_body_frd
        )
        x_frd, y_frd, z_frd = point_frd
        body_range = math.sqrt(x_frd * x_frd + y_frd * y_frd + z_frd * z_frd)
        if body_range < min_range or body_range > max_range:
            continue

        yaw_deg = math.degrees(math.atan2(y_frd, x_frd)) % 360.0
        horizontal_range = math.hypot(x_frd, y_frd)
        pitch_deg = math.degrees(math.atan2(-z_frd, horizontal_range))
        cell = (
            _angle_bin(yaw_deg, yaw_bin_deg, 360.0),
            int(math.floor((pitch_deg + 90.0) / pitch_bin_deg)),
        )
        previous = nearest_by_cell.get(cell)
        if previous is None or body_range < previous[0]:
            nearest_by_cell[cell] = (body_range, point_frd)

    candidates = [
        item[1] for item in sorted(nearest_by_cell.values(), key=lambda item: item[0])
    ]
    return candidates[:max_points]


def send_obstacle_distance_3d(
    mav,
    points_body_frd: Sequence[Point],
    *,
    timestamp_ms: Optional[int] = None,
    min_distance: float = LIDAR_MIN_M,
    max_distance: float = LIDAR_MAX_M,
) -> int:
    """Send one MAVLink 2 OBSTACLE_DISTANCE_3D per point.

    Every point from one scan uses one timestamp. AP_Proximity_MAV uses that
    timestamp to commit the temporary Boundary_3D after the scan ends.
    """

    mavutil = load_mavutil()
    if timestamp_ms is None:
        timestamp_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF

    for x_frd, y_frd, z_frd in points_body_frd:
        mav.mav.obstacle_distance_3d_send(
            int(timestamp_ms) & 0xFFFFFFFF,
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            0xFFFF,  # obstacle id unknown; let ArduPilot treat every point as new
            float(x_frd),
            float(y_frd),
            float(z_frd),
            float(min_distance),
            float(max_distance),
        )
    return len(points_body_frd)


def _read_ros_points(msg, point_cloud2, max_raw_points: int) -> List[Point]:
    points = []
    try:
        iterator = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        for point in iterator:
            points.append((float(point[0]), float(point[1]), float(point[2])))
            if len(points) >= max_raw_points:
                break
    except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
        raise RuntimeError(f"không đọc được PointCloud2: {exc}") from exc
    return points


def run_ros_backend(
    mav_target: str,
    ros_topic: str,
    *,
    max_points: int,
    max_raw_points: int,
    send_hz: float,
    sensor_offset_body_frd: Point,
    z_min: Optional[float],
    z_max: Optional[float],
) -> None:
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2
    except ImportError as exc:
        raise SystemExit(
            "[error] ROS2 backend cần rclpy, sensor_msgs và sensor_msgs_py. "
            "Nếu không cần ROS, thử backend --gz."
        ) from exc

    mav, _ = open_mavlink(mav_target)
    print(f"[lidar_bridge] ROS2: {ros_topic} -> {mav_target}")
    print(
        "[lidar_bridge] FLU -> BODY_FRD, "
        f"sensor offset={sensor_offset_body_frd}, max_points={max_points}"
    )

    rclpy.init()
    node = Node("lidar_to_mavlink_avoidance")
    last_send = 0.0
    frame_number = 0

    def callback(msg: PointCloud2) -> None:
        nonlocal last_send, frame_number
        now = time.monotonic()
        if now - last_send < 1.0 / send_hz:
            return
        last_send = now
        frame_number += 1

        try:
            raw_points = _read_ros_points(msg, point_cloud2, max_raw_points)
            obstacles = point_cloud_to_obstacles(
                raw_points,
                max_points=max_points,
                sensor_offset_body_frd=sensor_offset_body_frd,
                z_min=z_min,
                z_max=z_max,
            )
            sent = send_obstacle_distance_3d(mav, obstacles)
        except RuntimeError as exc:
            node.get_logger().error(str(exc))
            return

        if sent:
            nearest = min(
                math.sqrt(x * x + y * y + z * z) for x, y, z in obstacles
            )
            node.get_logger().info(
                f"frame={frame_number} raw={len(raw_points)} "
                f"points={sent} nearest={nearest:.2f}m"
            )
        else:
            # An empty scan is not converted into a fake obstacle. ArduPilot
            # will mark the MAV proximity backend stale after its 500 ms
            # timeout, which is safer than polluting the OA database.
            node.get_logger().warning(
                f"frame={frame_number} raw={len(raw_points)} points=0"
            )

    node.create_subscription(PointCloud2, ros_topic, callback, qos_profile_sensor_data)
    print("[lidar_bridge] spinning; Ctrl+C để thoát...")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _read_gz_points(msg, max_raw_points: int) -> List[Point]:
    """Decode gz.msgs.PointCloudPacked for the Harmonic Python bindings."""

    field_map = {field.name: field for field in msg.field}
    required = ("x", "y", "z")
    if any(name not in field_map for name in required):
        raise RuntimeError("PointCloudPacked không có đủ field x/y/z")

    # gz.msgs.PointCloudPacked.Field.DataType: FLOAT32=6, FLOAT64=7.
    formats = {6: ("f", 4), 7: ("d", 8)}
    for name in required:
        if int(field_map[name].datatype) not in formats:
            raise RuntimeError(f"field {name} không phải FLOAT32/FLOAT64")

    width = int(msg.width)
    height = int(msg.height)
    point_step = int(msg.point_step)
    row_step = int(msg.row_step) or width * point_step
    if width <= 0 or height <= 0 or point_step <= 0:
        return []

    endian = ">" if msg.is_bigendian else "<"
    points = []
    for index in range(min(width * height, max_raw_points)):
        row, column = divmod(index, width)
        base = row * row_step + column * point_step
        values = []
        valid = True
        for name in required:
            field = field_map[name]
            fmt, _ = formats[int(field.datatype)]
            try:
                value = struct.unpack_from(
                    endian + fmt, msg.data, base + int(field.offset)
                )[0]
            except struct.error:
                valid = False
                break
            values.append(float(value))
        if valid and all(math.isfinite(value) for value in values):
            points.append((values[0], values[1], values[2]))
    return points


def run_gz_backend(
    mav_target: str,
    gz_topic: str,
    *,
    max_points: int,
    max_raw_points: int,
    send_hz: float,
    sensor_offset_body_frd: Point,
    z_min: Optional[float],
    z_max: Optional[float],
) -> None:
    try:
        import gz.transport13 as gz_transport
        from gz.msgs10 import pointcloud_packed_pb2 as pc_pb2
    except ImportError as exc:
        raise SystemExit(
            "[error] GZ backend cần gz.transport13 và "
            "gz.msgs10.pointcloud_packed_pb2 phù hợp với Gazebo Harmonic. "
            "Dùng --ros nếu đã chạy ros_gz_bridge."
        ) from exc

    mav, _ = open_mavlink(mav_target)
    print(f"[lidar_bridge] Gazebo: {gz_topic} -> {mav_target}")
    node = gz_transport.Node()
    latest_lock = threading.Lock()
    latest_scan = None
    received_number = 0
    frame_number = 0
    last_source_number = 0

    def callback(msg) -> None:
        """Keep only the newest scan so processing can never build a backlog."""

        nonlocal latest_scan, received_number
        payload = msg.SerializeToString()
        with latest_lock:
            received_number += 1
            latest_scan = (received_number, payload)

    def process_latest_scan() -> bool:
        nonlocal latest_scan, frame_number, last_source_number
        with latest_lock:
            pending_scan = latest_scan
            latest_scan = None

        if pending_scan is None:
            return False

        source_number, payload = pending_scan
        msg = pc_pb2.PointCloudPacked()
        msg.ParseFromString(payload)
        frame_number += 1
        dropped = max(0, source_number - last_source_number - 1)
        last_source_number = source_number
        try:
            raw_points = _read_gz_points(msg, max_raw_points)
            obstacles = point_cloud_to_obstacles(
                raw_points,
                max_points=max_points,
                sensor_offset_body_frd=sensor_offset_body_frd,
                z_min=z_min,
                z_max=z_max,
            )
            sent = send_obstacle_distance_3d(mav, obstacles)
        except RuntimeError as exc:
            print(f"[gz] {exc}")
            return True
        if obstacles:
            nearest = min(
                math.sqrt(x * x + y * y + z * z) for x, y, z in obstacles
            )
            print(
                f"[gz] frame={frame_number} raw={len(raw_points)} "
                f"points={sent} nearest={nearest:.2f}m dropped={dropped}"
            )
        else:
            print(
                f"[gz] frame={frame_number} raw={len(raw_points)} "
                f"points=0 dropped={dropped}"
            )
        return True

    if not node.subscribe(pc_pb2.PointCloudPacked, gz_topic, callback):
        raise SystemExit(
            f"[error] không subscribe được {gz_topic}; kiểm tra GZ_PARTITION"
        )

    print("[lidar_bridge] running; Ctrl+C để thoát...")
    try:
        while True:
            started = time.monotonic()
            if not process_latest_scan():
                time.sleep(0.01)
                continue
            # The Gazebo callback keeps replacing latest_scan while this
            # frame is decoded and sent.  Waiting here controls output rate
            # without ever forcing new scans to queue behind old ones.
            remaining = (1.0 / send_hz) - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass


def run_distance_sensor_demo(mav_target: str) -> None:
    """Small fallback for testing the MAV proximity receiver without Gazebo."""

    mav, mavutil = open_mavlink(mav_target)
    print(f"[demo] gửi DISTANCE_SENSOR giả -> {mav_target}")
    try:
        orientation = 0
        while True:
            distance = 3.0 + 2.0 * math.sin(time.monotonic())
            mav.mav.distance_sensor_send(
                int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
                int(LIDAR_MIN_M * 100),
                int(LIDAR_MAX_M * 100),
                int(distance * 100),
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                0,
                orientation,
                0,
            )
            orientation = (orientation + 1) % 8
            time.sleep(0.12)
    except KeyboardInterrupt:
        pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Gazebo 3D LiDAR -> MAVLink 2 ArduPilot avoidance bridge"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--ros", action="store_true", help="nhận ROS 2 PointCloud2")
    mode.add_argument("--gz", action="store_true", help="nhận gz.msgs.PointCloudPacked")
    mode.add_argument("--demo", action="store_true", help="gửi DISTANCE_SENSOR giả")
    parser.add_argument(
        "--mav",
        default="tcp:127.0.0.1:5762",
        help="MAVLink target (mặc định là SITL SERIAL1 tcp:127.0.0.1:5762)",
    )
    parser.add_argument(
        "--topic",
        default="/sensor_suite/lidar/points",
        help="topic LiDAR đầu vào",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=DEFAULT_MAX_POINTS,
        help="số điểm 3D tối đa gửi mỗi frame (mặc định: 180)",
    )
    parser.add_argument(
        "--send-hz",
        type=float,
        default=DEFAULT_SEND_HZ,
        help="tần số gửi scan MAVLink (mặc định: 5 Hz)",
    )
    parser.add_argument(
        "--max-raw-points",
        type=int,
        default=25000,
        help="giới hạn điểm đọc từ một PointCloud2 (mặc định: 25000)",
    )
    parser.add_argument(
        "--sensor-offset-body-frd",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=DEFAULT_SENSOR_OFFSET_BODY_FRD,
        help="offset gốc LiDAR so với base_link trong BODY_FRD; "
        "model hiện tại: 0.08 0 -0.16",
    )
    parser.add_argument(
        "--z-min",
        type=float,
        default=None,
        help="bỏ điểm có z FLU nhỏ hơn giá trị này (mặc định: không lọc)",
    )
    parser.add_argument(
        "--z-max",
        type=float,
        default=None,
        help="bỏ điểm có z FLU lớn hơn giá trị này (mặc định: không lọc)",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    offset = tuple(args.sensor_offset_body_frd)

    if args.send_hz <= 0.0:
        raise SystemExit("[error] --send-hz phải lớn hơn 0")

    if args.demo:
        run_distance_sensor_demo(args.mav)
        return

    if not args.ros and not args.gz:
        try:
            import rclpy  # noqa: F401
            args.ros = True
        except ImportError:
            args.gz = True

    backend_args = dict(
        max_points=args.max_points,
        max_raw_points=args.max_raw_points,
        send_hz=args.send_hz,
        sensor_offset_body_frd=offset,
        z_min=args.z_min,
        z_max=args.z_max,
    )
    if args.ros:
        run_ros_backend(args.mav, args.topic, **backend_args)
    else:
        run_gz_backend(args.mav, args.topic, **backend_args)


if __name__ == "__main__":
    main()
