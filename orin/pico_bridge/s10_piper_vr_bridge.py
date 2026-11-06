#!/usr/bin/env python3
"""PXREA Pico bridge for the S10 S10/Piper + Piper whole-body policy.

PXREA runs on Orin, where roboticsservice and the Pico connection are already
available. Only a fixed binary command is relayed to the VR frontend on host
103. No browser or XLeVR service is used.
"""

import argparse
import ctypes
import errno
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import sys
import tempfile
import threading
import time


PACKET = struct.Struct("<7fi")
ARM_HOME = (0.42, 0.0, 0.05)
# Normal startup: ./run_vr_bridge.sh (no extra command-line arguments).
DEFAULT_ROBOT_IP = "10.21.41.1"
DEFAULT_ARM_SCALE = 1.0
DEFAULT_ARM_MAX_SPEED_M_S = 0.30
DEFAULT_VR_BASE = True
DEFAULT_RATE_HZ = 100.0
GRIP_THRESHOLD = 0.5
MAX_VR_STEP_M = 0.10
MAX_ARM_SLEW_DT_S = 0.05
INPUT_TIMEOUT_S = 0.30

PXREA_SERVER_CONNECT = 1 << 2
PXREA_SERVER_DISCONNECT = 1 << 3
PXREA_DEVICE_FIND = 1 << 4
PXREA_DEVICE_MISSING = 1 << 5
PXREA_DEVICE_CONNECT = 1 << 9
PXREA_DEVICE_STATE_JSON = 1 << 25
PXREA_FULL_MASK = 0xFFFFFFFF


class PXREADevStateJson(ctypes.Structure):
    _fields_ = [("devID", ctypes.c_char * 32), ("stateJson", ctypes.c_char * 16352)]


PXREACallback = ctypes.CFUNCTYPE(
    None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p
)


def finite_vector(values, length):
    if values is None or len(values) != length:
        return False
    return all(math.isfinite(float(value)) for value in values)


def parse_pose(pose_text):
    values = tuple(float(value) for value in pose_text.split(","))
    if len(values) < 7 or not finite_vector(values[:7], 7):
        raise ValueError("controller pose must contain seven finite values")
    return values[:7]


def vr_delta_to_robot(delta, right_xz=(1.0, 0.0)):
    """Rotate the tracking horizontal plane; operator right is robot -Y."""
    if not finite_vector(delta, 3):
        raise ValueError("VR delta must contain three finite values")
    rx, rz = normalize_right_axis(right_xz)
    x, y, z = (float(value) for value in delta)
    return (rz * x - rx * z, -rx * x - rz * z, y)


def normalize_right_axis(values):
    if not finite_vector(values, 2):
        raise ValueError("right_xz must contain two finite values")
    values = tuple(float(value) for value in values)
    norm = math.hypot(*values)
    if norm < 1.0e-6:
        raise ValueError("right_xz must not be zero")
    return tuple(float(value) / norm for value in values)


def load_right_axis(path):
    with Path(path).open() as handle:
        profile = json.load(handle)
    if not isinstance(profile, dict) or profile.get("version") != 1:
        raise ValueError("unsupported VR calibration version")
    return normalize_right_axis(profile["right_xz"])


def calibration_trial(positions):
    """Validate one outward rightward stroke, ending still before release."""
    if len(positions) < 8 or any(not finite_vector(p, 3) for p in positions):
        raise ValueError("样本不足，请按住Grip向右平移后稳住再松开")
    origin = positions[0]
    tail = positions[-8:]
    endpoint = tuple(sum(p[i] for p in tail) / len(tail) for i in range(3))
    delta = tuple(b - a for a, b in zip(origin, endpoint))
    horizontal = math.hypot(delta[0], delta[2])
    if not 0.08 <= horizontal <= 0.50:
        raise ValueError("请向右平移10～20cm；松开Grip后再移回，不要按住往返")
    if abs(delta[1]) > 0.35 * horizontal:
        raise ValueError("高度变化过大，请保持手柄水平")
    if max(math.dist(p, endpoint) for p in tail) > 0.015:
        raise ValueError("终点尚未稳定，请稳住约1秒后再松开Grip")
    maximum = max(math.dist(p, origin) for p in positions)
    if maximum > math.dist(endpoint, origin) * 1.35:
        raise ValueError("检测到折返；请松开Grip后再移回起点")
    if any(math.dist(a, b) > MAX_VR_STEP_M for a, b in
           zip(positions, positions[1:])):
        raise ValueError("检测到跟踪跳变，请重新校准")
    return normalize_right_axis((delta[0], delta[2]))


def combine_calibration_trials(first, second):
    first, second = normalize_right_axis(first), normalize_right_axis(second)
    if sum(a * b for a, b in zip(first, second)) < math.cos(math.radians(12)):
        raise ValueError("两次方向不一致；两次都要向右，松开Grip后再移回")
    return normalize_right_axis(tuple(a + b for a, b in zip(first, second)))


def save_right_axis(path, axis):
    """Publish a complete profile atomically; failed trials never replace it."""
    path = Path(path)
    axis = normalize_right_axis(axis)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", prefix=".vr_axes_",
                suffix=".json", dir=path.parent, delete=False) as handle:
            temporary = Path(handle.name)
            json.dump({"version": 1, "right_xz": axis}, handle)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class SideLockedClutch:
    """Intent hysteresis in hand metres; suppressed motion is never replayed."""

    ENTER_M = 0.006
    SWITCH_M = 0.012
    REFRESH_M = 0.008
    SWITCH_RATIO = 1.8

    def __init__(self, right_xz, scale):
        self.right_xz = normalize_right_axis(right_xz)
        if not math.isfinite(scale) or not 0.1 <= scale <= 4.0:
            raise ValueError("invalid clutch scale")
        self.scale = scale
        self.reset()

    def reset(self):
        self.mode = "WAIT"
        self.previous = None
        self.reference = None
        self.target = ARM_HOME

    def begin(self, position, target, previous_sample=None):
        if not finite_vector(position, 3) or not finite_vector(target, 3):
            raise ValueError("clutch anchor must be finite")
        if previous_sample is not None and not finite_vector(previous_sample, 3):
            raise ValueError("previous tracking sample must be finite")
        self.reset()
        self.reference = tuple(position)
        # A B-frame origin can precede first Grip engagement. Tracking-jump
        # detection must compare consecutive received poses, not distance
        # from that intentionally stored coordinate origin.
        self.previous = tuple(position if previous_sample is None else previous_sample)
        self.target = tuple(target)

    def update(self, position):
        if self.previous is None:
            raise ValueError("clutch must be anchored first")
        if not finite_vector(position, 3):
            raise ValueError("clutch position must be finite")
        position = tuple(float(value) for value in position)
        step = tuple(b - a for a, b in zip(self.previous, position))
        if math.dist(position, self.previous) > MAX_VR_STEP_M:
            raise ValueError("invalid pose or tracking jump")
        recent = vr_delta_to_robot(tuple(b - a for a, b in
                                   zip(self.reference, position)), self.right_xz)
        increment = vr_delta_to_robot(step, self.right_xz)
        side, xz = abs(recent[1]), max(abs(recent[0]), abs(recent[2]))
        if self.mode == "WAIT":
            if max(side, xz) >= self.ENTER_M:
                if max(side, xz) >= 1.4 * min(side, xz) or max(side, xz) >= self.SWITCH_M:
                    self.mode = "Y" if side >= xz else "XZ"
                    increment = recent
                    self.reference = tuple(position)
        else:
            active, other = (side, xz) if self.mode == "Y" else (xz, side)
            if other >= self.SWITCH_M and other >= self.SWITCH_RATIO * active:
                self.mode = "XZ" if self.mode == "Y" else "Y"
                # Use only this new sample, NOT accumulated suppressed motion.
                self.reference = tuple(position)
            elif active >= self.REFRESH_M and active >= 0.8 * other:
                self.reference = tuple(position)
        if self.mode == "Y":
            increment = (0.0, increment[1], 0.0)
        elif self.mode == "XZ":
            increment = (increment[0], 0.0, increment[2])
        else:
            increment = (0.0, 0.0, 0.0)
        self.target = add_scaled_clamped(self.target, increment, self.scale)
        self.previous = tuple(position)
        return self.target


class VrArmInput:
    """B establishes a new position origin; ordinary Grip remains a clutch.

    B's first-use origin survives a released Grip. During B hold, targets stay
    Home and the wire Home request remains asserted. Direction calibration
    does not change. Stale input / tracking jumps discard pending motion.
    """

    def __init__(self, right_xz, scale):
        self.clutch = SideLockedClutch(right_xz, scale)
        self.arm_target = ARM_HOME
        self.frame_origin = None
        self.pending_home_origin = None
        self.last_position = None
        self.last_sequence = None
        self.home_button_last = False
        self.grip_engaged = False
        self.event = None

    def input_lost(self):
        self.pending_home_origin = None
        self.last_position = None
        self.last_sequence = None
        self.grip_engaged = False
        self.clutch.reset()

    def reset_session(self):
        """Discard all targets and controller anchors from the last 103 run."""
        self.input_lost()
        self.arm_target = ARM_HOME
        self.frame_origin = None
        self.home_button_last = False
        self.event = None

    def update(self, right, sequence):
        position = tuple(right["pose"][:3])
        if not finite_vector(position, 3):
            raise ValueError("controller position must be finite")
        button = bool(right["secondary_button"])
        pressed = button and not self.home_button_last
        self.home_button_last = button
        grip = right["grip"] > GRIP_THRESHOLD
        self.event = None

        if pressed:
            self.frame_origin = self.pending_home_origin = position
            self.arm_target = ARM_HOME
            self.grip_engaged = False
            self.clutch.begin(position, ARM_HOME)
            self.event = "HOME_RECENTER"
        elif button:
            # Do not overwrite B's origin as the hand moves while B is held.
            self.arm_target = ARM_HOME
        elif (self.last_position is not None and
              math.dist(position, self.last_position) > MAX_VR_STEP_M):
            self.pending_home_origin = None
            self.clutch.begin(position, self.arm_target)
            self.grip_engaged = grip
            self.event = "POSE_JUMP"
        elif grip:
            if not self.grip_engaged:
                origin = (self.pending_home_origin if self.pending_home_origin
                          is not None else position)
                previous = (self.last_position if self.pending_home_origin
                            is not None else None)
                self.clutch.begin(origin, self.arm_target, previous_sample=previous)
                self.pending_home_origin = None
                self.grip_engaged = True
                self.event = "GRIP"
            if sequence != self.last_sequence:
                previous_mode = self.clutch.mode
                self.arm_target = self.clutch.update(position)
                if self.clutch.mode != previous_mode:
                    self.event = "MODE"
        else:
            self.grip_engaged = False
            self.clutch.reset()
        self.last_position = position
        self.last_sequence = sequence
        return self.arm_target, button


def add_scaled_clamped(anchor, robot_delta, scale):
    bounds = ((-0.42, 0.58), (-0.50, 0.50), (-0.30, 0.42))
    return tuple(max(bounds[index][0], min(bounds[index][1],
        float(anchor[index]) + float(robot_delta[index]) * scale))
        for index in range(3))


def move_towards_vector(current, target, max_distance):
    """Move a Cartesian command toward its target without a position jump."""
    if not finite_vector(current, 3) or not finite_vector(target, 3):
        raise ValueError("Cartesian slew endpoints must be finite 3-vectors")
    if not math.isfinite(max_distance) or max_distance < 0.0:
        raise ValueError("Cartesian slew distance must be finite and non-negative")
    current = tuple(float(value) for value in current)
    target = tuple(float(value) for value in target)
    distance = math.dist(current, target)
    if distance <= max_distance or distance < 1.0e-12:
        return target
    ratio = max_distance / distance
    return tuple(a + (b - a) * ratio for a, b in zip(current, target))


def gripper_opening_from_trigger(trigger):
    """Rest/release closes; pressing the right Trigger opens the gripper."""
    return 0.08 if float(trigger) > 0.5 else 0.0


def base_command_from_left(left):
    """Left stick owns both base X and yaw; missing left input stops both."""
    if left is None:
        return 0.0, 0.0
    return (max(-0.8, min(0.8, left["axis_y"])),
            max(-1.0, min(1.0, -left["axis_x"])))


class PicoState:
    def __init__(self):
        self._lock = threading.Lock()
        self._right = None
        self._left = None
        self._updated_at = 0.0
        self._sequence = 0

    @staticmethod
    def _controller(data):
        result = {
            "pose": parse_pose(data["pose"]),
            "grip": float(data.get("grip", 0.0)),
            "trigger": float(data.get("trigger", 0.0)),
            "axis_x": float(data.get("axisX", 0.0)),
            "axis_y": float(data.get("axisY", 0.0)),
            "secondary_button": bool(data.get("secondaryButton", False)),
        }
        if not finite_vector(
            (result["grip"], result["trigger"], result["axis_x"], result["axis_y"]),
            4,
        ):
            raise ValueError("controller input contains a non-finite value")
        return result

    def update_json(self, state_json):
        outer = json.loads(state_json)
        value = outer.get("value")
        if isinstance(value, str):
            value = json.loads(value)
        if not isinstance(value, dict):
            return
        controllers = value.get("Controller")
        if not isinstance(controllers, dict) or "right" not in controllers:
            return

        right = self._controller(controllers["right"])
        left = self._controller(controllers["left"]) if "left" in controllers else None
        with self._lock:
            self._right = right
            self._left = left
            self._updated_at = time.monotonic()
            self._sequence += 1

    def snapshot(self):
        with self._lock:
            return self._right, self._left, self._updated_at, self._sequence

    def clear_commands(self):
        """Require a fresh Pico callback before controlling a new 103 run."""
        with self._lock:
            self._right = None
            self._left = None
            self._updated_at = 0.0
            self._sequence += 1


def default_sdk_candidates():
    candidates = []
    configured = os.getenv("PXREA_SDK_LIB")
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.extend(
        [
            Path("/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so"),
            Path("/opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so"),
        ]
    )
    return candidates


def resolve_sdk_library(requested, script_dir):
    if requested:
        path = Path(requested).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"PXREA SDK library not found: {path}")
        return path
    for candidate in default_sdk_candidates():
        if candidate.is_file():
            return candidate.resolve()
    searched = "\n  ".join(str(path) for path in default_sdk_candidates())
    raise FileNotFoundError(
        "PXREA SDK library was not found. Set PXREA_SDK_LIB. Searched:\n  " + searched
    )


def load_pxrea(path, pico_state):
    sdk = ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
    sdk.PXREAInit.argtypes = [ctypes.c_void_p, PXREACallback, ctypes.c_uint]
    sdk.PXREAInit.restype = ctypes.c_int
    sdk.PXREADeinit.argtypes = []
    sdk.PXREADeinit.restype = ctypes.c_int

    @PXREACallback
    def callback(_context, event_type, status, user_data):
        try:
            if event_type == PXREA_SERVER_CONNECT:
                print("[PXREA] server connected")
            elif event_type == PXREA_SERVER_DISCONNECT:
                print("[PXREA] server disconnected")
            elif event_type in (
                PXREA_DEVICE_FIND, PXREA_DEVICE_MISSING, PXREA_DEVICE_CONNECT
            ):
                device = ""
                if user_data:
                    value = ctypes.cast(user_data, ctypes.c_char_p).value
                    if value:
                        device = value.decode("utf-8", errors="replace")
                print(f"[PXREA] event={event_type} status={status} device={device}")
            elif event_type == PXREA_DEVICE_STATE_JSON and user_data:
                message = ctypes.cast(
                    user_data, ctypes.POINTER(PXREADevStateJson)
                ).contents
                if message.stateJson:
                    pico_state.update_json(
                        message.stateJson.decode("utf-8", errors="strict")
                    )
        except Exception as error:  # callbacks must never unwind into C++
            print(f"[PXREA] ignored malformed callback: {error}", file=sys.stderr)

    result = sdk.PXREAInit(None, callback, PXREA_FULL_MASK)
    if result != 0:
        raise RuntimeError(f"PXREAInit failed with code {result}")
    return sdk, callback


def connect_robot(destination):
    connection = socket.create_connection(destination, timeout=1.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.settimeout(1.0)
    print(f"[PXREA Bridge] connected to 103 at {destination[0]}:{destination[1]}")
    return connection


def strategy_session_ended(connection):
    """Detect the 103 shutdown byte or TCP EOF without delaying VR frames."""
    try:
        marker = connection.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT)
    except (BlockingIOError, InterruptedError, socket.timeout):
        return False
    except OSError:
        return True
    return marker in (b"", b"R")


def parse_args():
    parser = argparse.ArgumentParser(
        description="PXREA Pico to S10 S10/Piper/Piper whole-body-policy bridge"
    )
    parser.add_argument(
        "--robot-ip",
        default=os.getenv("S10_PIPER_VR_ROBOT_IP", DEFAULT_ROBOT_IP),
        help="103 host address (default: 10.21.41.1)",
    )
    parser.add_argument("--port", type=int, default=8089)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument(
        "--scale", type=float, default=DEFAULT_ARM_SCALE,
        help="Piper target metres per controller metre (default: 1.0)",
    )
    parser.add_argument(
        "--arm-max-speed", type=float, default=DEFAULT_ARM_MAX_SPEED_M_S,
        help="maximum Cartesian command slew speed in m/s (default: 0.30)",
    )
    parser.add_argument("--sdk-lib", help="path to libPXREARobotSDK.so")
    parser.add_argument(
        "--vr-base", action="store_true", default=DEFAULT_VR_BASE,
        help="forward Pico sticks (default: on; 103 must set S10_PIPER_VR_BASE=1)",
    )
    parser.add_argument("--no-vr-base", dest="vr_base", action="store_false",
                        help="optional: disable forwarding base stick commands")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--calibrate-right", action="store_true",
        help="read-only two-stroke rightward calibration; never connects to 103")
    parser.add_argument("--calibration-file", type=Path,
        default=Path(__file__).resolve().parent / "vr_axes.json")
    parser.add_argument("--calibration-timeout", type=float, default=60.0)
    return parser.parse_args()


def run_calibration(args):
    if not math.isfinite(args.calibration_timeout) or not 10 <= args.calibration_timeout <= 180:
        raise SystemExit("--calibration-timeout must be in [10, 180] seconds")
    print("[校准/只读] 不连接103，不发送底盘、机械臂或夹爪指令。请先退出其他遥操作程序并保持机械臂失能。")
    print("[校准] 朝向保持不变：松开右Grip稳住手柄；按住Grip水平向右移10～20cm，稳住1秒再松开；松开后移回。重复两次，两次都向右。")
    state = PicoState()
    sdk, callback_keepalive = load_pxrea(
        resolve_sdk_library(args.sdk_lib, Path(__file__).resolve().parent), state)
    deadline = time.monotonic() + args.calibration_timeout
    sequence_last = 0
    armed = False
    positions = None
    trials = []
    try:
        while time.monotonic() < deadline:
            right, _left, updated, sequence = state.snapshot()
            if right is None or time.monotonic() - updated > INPUT_TIMEOUT_S:
                if positions is not None:
                    print("[校准] 输入中断，本次动作作废；松开Grip后重试。")
                armed = False
                positions = None
                time.sleep(0.02)
                continue
            if sequence == sequence_last:
                time.sleep(0.005)
                continue
            sequence_last = sequence
            grip = right["grip"] > GRIP_THRESHOLD
            if not grip:
                armed = True
                if positions is None:
                    continue
                try:
                    axis = calibration_trial(positions)
                    if trials:
                        calibrated = combine_calibration_trials(trials[0], axis)
                        save_right_axis(args.calibration_file, calibrated)
                        print(f"[校准成功] right_xz={calibrated}; 保存到 {args.calibration_file}")
                        print("[校准成功] 朝向改变或Pico重新居中后需要重做；现在可单独启动VR桥接。")
                        return
                    trials.append(axis)
                    print("[校准] 第1次通过。松开Grip移回起点，再按住向右平移一次。")
                except ValueError as error:
                    trials.clear()
                    print(f"[校准未通过] {error}；请重新做两次。")
                finally:
                    positions = None
            elif positions is not None:
                positions.append(tuple(right["pose"][:3]))
            elif armed:
                positions = [tuple(right["pose"][:3])]
                armed = False
                print(f"[校准] 记录第{len(trials)+1}次：只向右平移，稳住后松开Grip。")
            time.sleep(0.005)
        raise SystemExit("校准超时；未修改已有校准文件。请重新运行 --calibrate-right。")
    finally:
        sdk.PXREADeinit()
        _ = callback_keepalive


def self_test():
    assert PACKET.size == 32
    assert ctypes.sizeof(PXREADevStateJson) == 16384
    assert gripper_opening_from_trigger(0.0) == 0.0
    assert gripper_opening_from_trigger(0.5) == 0.0
    assert gripper_opening_from_trigger(1.0) == 0.08
    assert vr_delta_to_robot((1.0, 2.0, 3.0)) == (-3.0, -1.0, 2.0)
    assert base_command_from_left(None) == (0.0, 0.0)
    assert base_command_from_left({"axis_x": 1.0, "axis_y": 0.4}) == (0.4, -1.0)
    assert base_command_from_left({"axis_x": -1.0, "axis_y": 2.0}) == (0.8, 1.0)
    assert add_scaled_clamped(ARM_HOME, (0.0, 0.0, 0.0), 1.0) == ARM_HOME
    assert add_scaled_clamped(ARM_HOME, (1.0, -1.0, 1.0), 1.0) == (
        0.58, -0.50, 0.42
    )
    assert move_towards_vector((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.1) == (
        0.0, 0.0, 0.0
    )
    slewed = move_towards_vector((0.0, 0.0, 0.0), (0.3, 0.4, 0.0), 0.1)
    assert all(abs(a - b) < 1.0e-12 for a, b in zip(slewed, (0.06, 0.08, 0.0)))
    assert move_towards_vector(slewed, (0.3, 0.4, 0.0), 1.0) == (0.3, 0.4, 0.0)
    mapped = add_scaled_clamped(
        ARM_HOME, vr_delta_to_robot((0.01, -0.02, 0.03)), 1.0
    )
    assert all(abs(a - b) < 1.0e-9 for a, b in zip(mapped, (0.39, -0.01, 0.03)))
    # Forward is negative Pico Z: at 1.0x, 10 cm forward adds 10 cm robot X.
    for hand_z, expected_x in ((-0.1, 0.52), (0.1, 0.32)):
        target = add_scaled_clamped(
            ARM_HOME, vr_delta_to_robot((0.0, 0.0, hand_z)), DEFAULT_ARM_SCALE)
        assert abs(target[0] - expected_x) < 1.0e-9
        assert target[1:] == ARM_HOME[1:]
    sample = {
        "value": json.dumps({
            "Controller": {
                "right": {
                    "pose": "1,2,3,0,0,0,1", "grip": 0.7,
                    "trigger": 0.4, "axisX": 0.1, "axisY": -0.2,
                }
            }
        })
    }
    state = PicoState()
    state.update_json(json.dumps(sample))
    right, _left, updated_at, sequence = state.snapshot()
    assert right["pose"][:3] == (1.0, 2.0, 3.0)
    assert updated_at > 0.0 and sequence == 1
    state.clear_commands()
    assert state.snapshot()[0:3] == (None, None, 0.0)
    arm_input = VrArmInput((1.0, 0.0), DEFAULT_ARM_SCALE)
    arm_input.arm_target = (0.55, 0.1, 0.1)
    arm_input.frame_origin = (1.0, 2.0, 3.0)
    arm_input.reset_session()
    assert arm_input.arm_target == ARM_HOME and arm_input.frame_origin is None
    bridge_socket, robot_socket = socket.socketpair()
    try:
        assert not strategy_session_ended(bridge_socket)
        robot_socket.sendall(b"R")
        assert strategy_session_ended(bridge_socket)
    finally:
        bridge_socket.close()
        robot_socket.close()
    bridge_socket, robot_socket = socket.socketpair()
    robot_socket.close()
    try:
        assert strategy_session_ended(bridge_socket)
    finally:
        bridge_socket.close()
    assert len(PACKET.pack(0.0, 0.0, 0.0, *ARM_HOME, 0.08, 0)) == 32
    clutch = SideLockedClutch((1.0, 0.0), 1.5)
    clutch.begin((0.0, 0.0, 0.0), ARM_HOME)
    for i in range(1, 50):
        target = clutch.update((i * 0.004, i * 0.0003, i * 0.0005))
        assert target[0] == ARM_HOME[0] and target[2] == ARM_HOME[2]
    assert clutch.mode == "Y" and target[1] < 0.0
    print("PXREA VR bridge self-test passed: JSON, clutch axes and wire protocol")


def run(args):
    if not (1.0 <= args.rate <= 200.0):
        raise SystemExit("--rate must be in [1, 200] Hz")
    if not (0.1 <= args.scale <= 4.0):
        raise SystemExit("--scale must be in [0.1, 4.0]")
    if not (0.01 <= args.arm_max_speed <= 1.0):
        raise SystemExit("--arm-max-speed must be in [0.01, 1.0] m/s")

    try:
        right_axis = load_right_axis(args.calibration_file)
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"VR方向未校准或校准无效：{error}。先保持机械臂失能，运行 ./run_vr_bridge.sh --calibrate-right")
    arm_input = VrArmInput(right_axis, args.scale)
    clutch = arm_input.clutch

    script_dir = Path(__file__).resolve().parent
    sdk_path = resolve_sdk_library(args.sdk_lib, script_dir)
    pico_state = PicoState()
    sdk, callback_keepalive = load_pxrea(sdk_path, pico_state)
    stop_requested = threading.Event()

    def request_stop(_signum, _frame):
        stop_requested.set()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)

    destination = (args.robot_ip, args.port)
    period = 1.0 / args.rate
    connection = None
    last_connect_attempt = 0.0
    arm_target = ARM_HOME
    commanded_arm_target = ARM_HOME
    last_arm_command_time = None
    home_return_pending = False
    stale_reported = False

    def clear_strategy_session(reason):
        nonlocal connection, arm_target, commanded_arm_target
        nonlocal last_arm_command_time, home_return_pending
        if connection is not None:
            connection.close()
            connection = None
        arm_input.reset_session()
        pico_state.clear_commands()
        arm_target = commanded_arm_target = ARM_HOME
        last_arm_command_time = None
        home_return_pending = False
        print(f"\n[PXREA Bridge] 103 {reason}; cleared VR arm/base targets and controller anchors")

    print(f"[PXREA Bridge] SDK={sdk_path}")
    print(f"[PXREA Bridge] arm displacement scale={args.scale:g}x")
    print(f"[PXREA Bridge] Cartesian target slew limit={args.arm_max_speed:g} m/s "
          f"({1000.0 * args.arm_max_speed / args.rate:g} mm/nominal frame)")
    print("[PXREA Bridge] no webpage; waiting for Pico controller callbacks")
    print("[PXREA Bridge] right Grip=clutch; right Trigger: press=OPEN, release/default=CLOSED")
    print("[PXREA Bridge] right B: Home + 本帧手柄位置设为新原点；按住B保持Home并停底盘，方向校准不变")
    print(f"[PXREA Bridge] calibrated operator-right axis (VR X/Z)={right_axis}; right -> -arm Y")
    print("[PXREA Bridge] 侧移锁定：Y模式冻结X/Z，仅J1；XZ模式冻结Y；切换不补发锁住期间的位移")
    print("[PXREA Bridge] intent threshold=6mm, switch=12mm/1.8x; release Grip to re-anchor")
    print("[PXREA Bridge] left stick: vertical=base X, horizontal=yaw; right stick does not steer")

    next_tick = time.monotonic()
    try:
        while not stop_requested.is_set():
            next_tick += period
            if connection is not None and strategy_session_ended(connection):
                clear_strategy_session("strategy session ended")
                time.sleep(max(0.0, next_tick - time.monotonic()))
                continue
            right, left, updated_at, sequence = pico_state.snapshot()
            now = time.monotonic()
            fresh = right is not None and now - updated_at <= INPUT_TIMEOUT_S
            if not fresh:
                arm_input.input_lost()
                last_arm_command_time = None
                if connection is not None:
                    connection.close()
                    connection = None
                if right is not None and not stale_reported:
                    print("\n[PXREA Bridge] input timeout; 103 freezes arm target")
                    stale_reported = True
                time.sleep(max(0.0, next_tick - time.monotonic()))
                continue
            stale_reported = False

            if connection is None and now - last_connect_attempt >= 1.0:
                last_connect_attempt = now
                try:
                    connection = connect_robot(destination)
                    last_arm_command_time = None
                except OSError as error:
                    print(f"\n[PXREA Bridge] waiting for 103: {error}")
            if connection is None:
                # Motion while 103 is offline must not accumulate into the
                # first target sent when the next strategy run connects.
                arm_input.input_lost()
                time.sleep(max(0.0, next_tick - time.monotonic()))
                continue

            grip = right["grip"] > GRIP_THRESHOLD
            arm_target, home_requested = arm_input.update(right, sequence)
            if arm_input.event == "HOME_RECENTER":
                home_return_pending = True
                print(f"\n[PXREA Bridge] B: Home + 新位置原点={arm_input.frame_origin}; "
                      "清除旧位移/侧移模式，渐进回Home")
            elif arm_input.event == "GRIP":
                print("\n[PXREA Bridge] right Grip engaged; Cartesian clutch anchored")
            elif arm_input.event == "MODE":
                print(f"\n[PXREA Bridge] intent={clutch.mode}; " +
                      ("X/Z冻结，仅侧移J1" if clutch.mode == "Y" else "Y冻结，控制X/Z"))
            elif arm_input.event == "POSE_JUMP":
                print("\n[PXREA Bridge] rejected >0.10 m pose jump; "
                      "held target and cleared pending B displacement")

            vx = 0.0
            wz = 0.0
            if args.vr_base:
                vx, wz = base_command_from_left(left)
            gripper_opening_m = gripper_opening_from_trigger(right["trigger"])

            if connection is not None:
                command_dt = (period if last_arm_command_time is None else
                              min(MAX_ARM_SLEW_DT_S,
                                  max(0.0, now - last_arm_command_time)))
                home_in_progress = home_requested or home_return_pending
                slew_destination = ARM_HOME if home_in_progress else arm_target
                commanded_arm_target = move_towards_vector(
                    commanded_arm_target, slew_destination,
                    args.arm_max_speed * command_dt)
                last_arm_command_time = now
                home_reached = math.dist(commanded_arm_target, ARM_HOME) < 1.0e-6
                home_wire_request = home_in_progress and home_reached
                try:
                    connection.sendall(PACKET.pack(
                        0.0 if home_in_progress else vx, 0.0,
                        0.0 if home_in_progress else wz, *commanded_arm_target,
                        gripper_opening_m, -1001 if home_wire_request else 0
                    ))
                    if home_wire_request and not home_requested:
                        home_return_pending = False
                except OSError as error:
                    print(f"\n[PXREA Bridge] send failed: {error}")
                    if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK,
                                       errno.ETIMEDOUT):
                        connection.close()
                        connection = None
                        last_arm_command_time = None
                    else:
                        clear_strategy_session("connection lost")
            else:
                # Do not consume a large Cartesian jump while 103 is offline.
                last_arm_command_time = None

            sys.stdout.write(
                f"\r[Pico] grip={'ON ' if grip else 'OFF'} "
                f"mode={clutch.mode} "
                f"XYZcmd=({commanded_arm_target[0]:+.3f},"
                f"{commanded_arm_target[1]:+.3f},"
                f"{commanded_arm_target[2]:+.3f}) gripper="
                f"{'close' if gripper_opening_m == 0.0 else 'open '}  "
            )
            sys.stdout.flush()
            time.sleep(max(0.0, next_tick - time.monotonic()))
    finally:
        print("\n[PXREA Bridge] stopped")
        if connection is not None:
            connection.close()
        sdk.PXREADeinit()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        # Keep the callback object reachable until PXREADeinit has completed.
        _ = callback_keepalive


def main():
    args = parse_args()
    if args.self_test:
        self_test()
        return
    if args.calibrate_right:
        run_calibration(args)
        return
    run(args)


if __name__ == "__main__":
    main()
