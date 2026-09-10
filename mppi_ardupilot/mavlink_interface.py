"""Nối MAVLink giữa node MPPI (edge computer) và ArduPilot.

ArduPilot ở GUIDED chỉ bám velocity setpoint; node gửi
``SET_POSITION_TARGET_LOCAL_NED`` mang ``vx, vy, vz, yaw_rate`` ở 10-20 Hz.
Nếu stream ngừng quá GUID_TIMEOUT (mặc định 3 s), copter tự brake và hold.

Về type_mask (đã đối chiếu 3 nguồn: MAVLink common.xml trong
``ardupilot/modules/mavlink``, handler
``GCS_MAVLINK_Copter::handle_message_set_position_target_local_ned``
và dialect pymavlink cài trong env):

    bit 0-2 (1|2|4)      : ignore position xyz
    bit 3-5 (8|16|32)    : ignore velocity xyz
    bit 6-8 (64|128|256) : ignore accel xyz
    bit 9 (512)          : FORCE_SET (accel hiểu thành force)
    bit 10 (1024)        : ignore yaw
    bit 11 (2048)        : ignore yaw rate

Dùng velocity + yaw_rate, bỏ position/accel/yaw:

    mask = 1|2|4 | 64|128|256 | 1024 = 1479

Bẫy thường gặp: nhầm bit 3-5 thành "position ignore" (thực ra là
velocity ignore) sẽ khiến ArduPilot BỎ velocity, đọc position (0,0,0)
và — vì vô tình set bit FORCE_SET trong khi accel không ignore —
rơi vào nhánh ``hold_position()``: drone đứng yên dù log node vẫn đẹp.
"""

import os
import time
from typing import Optional, Tuple

import numpy as np

# Dùng velocity + yaw_rate, ignore position/accel/yaw. Dựng từ bit có tên
# để không còn magic number trôi nổi trong code.
TYPE_MASK_VEL_YAWRATE = (
    1 | 2 | 4  # ignore position xyz
    | 64 | 128 | 256  # ignore accel xyz
    | 1024  # ignore yaw, GIỮ yaw_rate
)
assert TYPE_MASK_VEL_YAWRATE == 1479, TYPE_MASK_VEL_YAWRATE

# Chỉ dùng velocity, bỏ cả yaw/yaw_rate (fallback khi chưa muốn lái yaw).
TYPE_MASK_VEL_ONLY = TYPE_MASK_VEL_YAWRATE | 2048
assert TYPE_MASK_VEL_ONLY == 3527, TYPE_MASK_VEL_ONLY

# SET_ATTITUDE_TARGET: ignore quaternion attitude, use all three body rates and
# thrust. ArduCopter rejects a partial body-rate vector, so bits 0..2 must all
# remain clear. GUID_OPTIONS bit 3 must be set for thrust-as-thrust semantics.
TYPE_MASK_BODY_RATES_THRUST = 1 << 7
assert TYPE_MASK_BODY_RATES_THRUST == 128

ATTITUDE_RATE_IGNORE_BITS = (1 << 0) | (1 << 1) | (1 << 2)


def validate_body_rate_thrust_mask(mask: int) -> None:
    """Reject masks incompatible with the target ArduCopter handler.

    Commit f808f78 accepts either all three body rates or none.  This project
    additionally requires all three rates, thrust and attitude-ignore.
    """
    mask = int(mask)
    ignored_rates = mask & ATTITUDE_RATE_IGNORE_BITS
    if ignored_rates not in (0, ATTITUDE_RATE_IGNORE_BITS):
        raise ValueError("ArduCopter từ chối body-rate vector chỉ có một phần")
    if ignored_rates != 0 or (mask & (1 << 6)) or not (mask & (1 << 7)):
        raise ValueError("project cần đủ 3 body rates + thrust, bỏ attitude")


def _load_mavutil():
    os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")
    os.environ["MAVLINK20"] = "1"
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise SystemExit(
            "[error] cần pymavlink (env ardupilot-rviz hoặc pip install pymavlink)."
        ) from exc
    mavutil.set_dialect("ardupilotmega")
    return mavutil


def parse_state(local_position_ned, attitude) -> Tuple[np.ndarray, np.ndarray, float]:
    """Tách (pos_ned, vel_ned, yaw) từ 2 message MAVLink (duck-typed).

    Tách thành hàm thuần để unit-test được mà không cần SITL: chỉ cần
    object có đủ field .x/.y/.z/.vx/.vy/.vz và .yaw.
    """
    pos = np.array(
        [float(local_position_ned.x), float(local_position_ned.y), float(local_position_ned.z)],
        dtype=np.float64,
    )
    vel = np.array(
        [float(local_position_ned.vx), float(local_position_ned.vy), float(local_position_ned.vz)],
        dtype=np.float64,
    )
    return pos, vel, float(attitude.yaw)


class ArduPilotInterface:
    """MAVLink endpoint của node MPPI: đọc telemetry, gửi velocity target."""

    def __init__(self, connection: str = "tcp:127.0.0.1:5762") -> None:
        self.mavutil = _load_mavutil()
        m = self.mavutil
        self.master = m.mavlink_connection(
            connection, source_system=255, source_component=191, dialect="ardupilotmega"
        )
        print(f"[mavlink] chờ heartbeat từ {connection} ...")
        if self.master.wait_heartbeat(timeout=10) is None:
            raise SystemExit(f"[error] không nhận được heartbeat từ {connection}")
        print(
            f"[mavlink] đã nối sys {self.master.target_system} "
            f"comp {self.master.target_component}"
        )
        self._lned = None
        self._att = None
        self._lned_t = 0.0
        self._att_t = 0.0

    def request_message_interval(self, message_id: int, hz: float) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            self.mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            int(1e6 / hz),
            0, 0, 0, 0, 0,
        )

    def setup_telemetry(self, hz: float = 20.0) -> None:
        """Xin LOCAL_POSITION_NED + ATTITUDE đều 20 Hz cho vòng MPPI."""
        m = self.mavutil.mavlink
        self.request_message_interval(m.MAVLINK_MSG_ID_LOCAL_POSITION_NED, hz)
        self.request_message_interval(m.MAVLINK_MSG_ID_ATTITUDE, hz)

    def require_parameter(self, name: str, expected: float, timeout_s: float = 3.0) -> float:
        """Read one ArduPilot parameter and fail before unsafe control output."""
        encoded = name.encode("ascii")
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component, encoded, -1
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is None:
                continue
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", errors="ignore")
            if str(param_id).rstrip("\x00") != name:
                continue
            value = float(msg.param_value)
            if abs(value - expected) > 1e-4:
                raise SystemExit(
                    f"[safety] {name}={value:g}, cần {expected:g} trước khi "
                    "gửi thrust/body-rate"
                )
            return value
        raise SystemExit(f"[safety] không đọc được parameter {name} từ ArduPilot")

    def spin_once(self, timeout: float = 0.0) -> None:
        """Hút message chờ sẵn vào cache mới nhất (không block quá timeout)."""
        m = self.mavutil.mavlink
        deadline = time.monotonic() + timeout
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                if time.monotonic() >= deadline:
                    return
                time.sleep(0.001)
                continue
            t = time.monotonic()
            if msg.get_msgId() == m.MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                self._lned, self._lned_t = msg, t
            elif msg.get_msgId() == m.MAVLINK_MSG_ID_ATTITUDE:
                self._att, self._att_t = msg, t
            if time.monotonic() >= deadline and msg is None:
                return

    def get_state(self, max_age_s: float = 1.0):
        """Trả (pos_ned, vel_ned, yaw) hoặc None khi telemetry thiếu/cũ."""
        now = time.monotonic()
        if (
            self._lned is None
            or self._att is None
            or (now - self._lned_t) > max_age_s
            or (now - self._att_t) > max_age_s
        ):
            return None
        return parse_state(self._lned, self._att)

    def get_state_age_s(self) -> float:
        """Age of the oldest telemetry component used by ``get_state``."""
        if self._lned is None or self._att is None:
            return float("inf")
        now = time.monotonic()
        return max(now - self._lned_t, now - self._att_t)

    def send_velocity_ned(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rate: float = 0.0,
        use_yaw_rate: bool = True,
    ) -> None:
        """Gửi velocity setpoint. NED: +x North, +y East, +z Down.

        Muốn bay LÊN thì vz (vd) phải ÂM. yaw_rate [rad/s], dương khi
        quay thuận kim đồng hồ nhìn từ trên (chuẩn NED).
        """
        m = self.mavutil.mavlink
        mask = TYPE_MASK_VEL_YAWRATE if use_yaw_rate else TYPE_MASK_VEL_ONLY
        self.master.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            m.MAV_FRAME_LOCAL_NED,
            mask,
            0.0, 0.0, 0.0,  # position: bị mask bỏ qua
            float(vn), float(ve), float(vd),  # velocity: dùng
            0.0, 0.0, 0.0,  # accel: bị mask bỏ qua
            0.0,  # yaw: bị mask bỏ qua
            float(yaw_rate),  # yaw_rate: dùng (khi mask 1479)
        )

    def send_attitude_target_body_rates(
        self,
        roll_rate_frd: float,
        pitch_rate_frd: float,
        yaw_rate_frd: float,
        thrust_normalized: float,
    ) -> None:
        """Send body FRD rates [rad/s] and normalized collective thrust.

        This requires ArduCopter ``GUID_OPTIONS=8``. With ``GUID_OPTIONS=0``
        the same MAVLink thrust field means climb rate instead, so callers must
        only enable this path with the dedicated SITL parameter override.
        """
        values = np.asarray(
            [roll_rate_frd, pitch_rate_frd, yaw_rate_frd, thrust_normalized],
            dtype=np.float64,
        )
        if not np.isfinite(values).all():
            raise ValueError("SET_ATTITUDE_TARGET chứa NaN/Inf")
        validate_body_rate_thrust_mask(TYPE_MASK_BODY_RATES_THRUST)
        thrust = float(np.clip(thrust_normalized, 0.0, 1.0))
        self.master.mav.set_attitude_target_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            TYPE_MASK_BODY_RATES_THRUST,
            [1.0, 0.0, 0.0, 0.0],  # ignored by mask bit 7
            float(roll_rate_frd),
            float(pitch_rate_frd),
            float(yaw_rate_frd),
            thrust,
        )


def check_mask_against_dialect() -> None:
    """Đối chiếu mask với enum của chính dialect đang cài (fail nhanh)."""
    m = _load_mavutil().mavlink
    expect = (
        m.POSITION_TARGET_TYPEMASK_X_IGNORE
        | m.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | m.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )
    assert expect == 1479, f"dialect đổi enum? mask={expect}"
    assert TYPE_MASK_VEL_YAWRATE == expect
    attitude_expect = m.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
    assert attitude_expect == TYPE_MASK_BODY_RATES_THRUST
    validate_body_rate_thrust_mask(attitude_expect)
