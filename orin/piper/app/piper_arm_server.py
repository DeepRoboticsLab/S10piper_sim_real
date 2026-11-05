#!/usr/bin/env python3
"""Fail-closed Piper joint bridge for the S10 whole-body controller.

Protocol: newline-delimited JSON over TCP. The server streams
{"type":"state","q":[rad...],"dq":[rad/s...]} and accepts
{"type":"joint","target":[rad...]},
{"type":"pose_offset","xyz_m":[m...],"pitch_rad":rad}, or
{"type":"link6_offset","xyz_m":[m...],"joint6_target_rad":rad}, or
{"type":"gripper","opening_m":m}, {"type":"hold"},
{"type":"home_disable"}, or {"type":"disable"}.  pose_offset uses Piper's
official MOVE_P IK and is
relative to a validated MOVE_P Home. link6_offset uses continuous numerical
IK from the whole-body policy's exact training Home and commands MOVE_J.
"""

import argparse
import json
import math
import os
import select
import socket
import subprocess
import sys
import termios
import threading
import time
import tty
from typing import List, Optional

import numpy as np
from piper_sdk import C_PiperForwardKinematics, C_PiperInterface_V2


JOINT_LIMITS_RAD = tuple(
    (math.radians(lo), math.radians(hi))
    for lo, hi in (
        (-150.0, 150.0),
        (0.0, 180.0),
        (-170.0, 0.0),
        (-100.0, 100.0),
        (-70.0, 70.0),
        (-120.0, 120.0),
    )
)
# Feedback is allowed a modest calibration margin around the official command
# limits, but impossible multi-turn samples must never be used as a homing
# start.  In particular, a transient J2=-267 degree sample was observed after
# an Orin reboot while CAN feedback subsequently returned to about -10 degree.
FEEDBACK_SANITY_LIMITS_RAD = tuple(
    (math.radians(lo), math.radians(hi))
    for lo, hi in (
        (-180.0, 180.0),
        (-15.0, 195.0),
        (-185.0, 15.0),
        (-120.0, 120.0),
        (-90.0, 90.0),
        # This Piper reports a firmware J6 range of [-180, 180] degrees.
        # Accept that measured startup range, while JOINT_LIMITS_RAD above
        # continues to restrict all policy targets to the standard +/-120.
        (-185.0, 185.0),
    )
)
# Startup must first latch the measured wrist without jumping from a valid
# firmware pose to the narrower policy limit.  Only J6 differs from the normal
# policy command limits, matching the limits queried from this physical arm.
STARTUP_HOLD_LIMITS_RAD = JOINT_LIMITS_RAD[:-1] + (
    (math.radians(-180.0), math.radians(180.0)),
)
RAD_TO_MILLI_DEG = 180000.0 / math.pi
# FK of the validated MOVE_P preparation joints
# [0, 15, -30, 0, -10, 0] degrees with dh_is_offset=1.
# A fixed reference prevents a TCP reconnect from adding the existing task
# offset to a newly captured current pose.
MOVE_P_HOME_POSE = [44.1214467, 0.0, 385.2777773, 0.0, 60.0, 0.0]
MOVE_P_SEND_PERIOD = 0.02  # official keyboard controller's proven 50 Hz
# The whole-body controller runs at 200 Hz, but one Piper JointCtrl command is
# expanded into several CAN frames.  Sending every controller tick can fill
# the gs_usb transmit path and make the SDK report SEND_MESSAGE_FAILED.
JOINT_SEND_PERIOD = 0.02  # 50 Hz, same proven rate as MOVE_P
# Training Home used by the original 22-DoF whole-body policy.  The numerical
# IK variant treats the position of link6 at this posture as the zero-offset
# pen tip; it deliberately does not use the separate MOVE_P preparation pose.
TRAINING_HOME_JOINTS = (0.0, 2.54, -1.26, 0.0, -1.22, 0.0)
TRAINING_HOME_LINK6_XYZ_M = (
    0.4586138634777516,
    0.0,
    0.028571078411618625,
)
# Named client workspaces are expressed as link6 offsets from training Home.
# The training workspace matches the policy target coordinates
# X[-0.42,0.58], Y[-0.30,0.30], Z[-0.30,0.42].
LINK6_WORKSPACE_OFFSET_BOUNDS_M = {
    "home_workspace": (
        (-0.06, 0.06), (-0.08, 0.08), (-0.09, 0.09)
    ),
    "training_workspace": (
        (-0.84, 0.16), (-0.30, 0.30), (-0.35, 0.37)
    ),
    # S10/Piper VR and layered clients explicitly opt into +/-0.50 m lateral input.
    "lateral_box": (
        (-0.84, 0.16), (-0.50, 0.50), (-0.35, 0.37)
    ),
}
LINK6_IK_SEND_PERIOD = 0.02
LINK6_IK_TOLERANCE_M = 0.0015
LINK6_IK_DAMPING = 0.025
LINK6_IK_FD_STEP_RAD = 1.0e-4
LINK6_IK_MAX_ITERATIONS = 12
# The established VR build keeps J4-J6 locked. The keyboard layered
# client explicitly opts into full-joint position IK.  Real Piper joint limits
# remain enforced in every mode; "full joint" only removes the software locks.
LINK6_IK_ACTIVE_JOINTS = (0, 1, 2)
LINK6_IK_LOCKED_JOINTS = (3, 4, 5)
TRAINING_J123_PROFILE = "training_j123_locked_j456_v1"
TRAINING_Y_J1_PROFILE = "training_y_joint1_xz_j23_v1"
TRAINING_J15_PROFILE = "training_j12345_locked_j6_v1"
OFFICIAL_MOVE_P_PROFILE = "training_home_move_p"
LINK6_IK_J5_ACTIVE_JOINTS = (0, 1, 2, 4)
LINK6_IK_J5_LOCKED_JOINTS = (3, 5)
LINK6_IK_ALL_ACTIVE_JOINTS = (0, 1, 2, 3, 4, 5)
LINK6_IK_ALL_LOCKED_JOINTS = ()
LINK6_IK_WRIST_YAW_ACTIVE_JOINTS = (0, 1, 2, 3, 4)
LINK6_IK_WRIST_YAW_LOCKED_JOINTS = (5,)
# Keyboard cylindrical control: Y owns only J1. X/Z may use J2-J5 while
# J1 remains at the angle derived only from Y and J6 remains independent.
LINK6_IK_J1_Y_ACTIVE_JOINTS = (1, 2, 3, 4)
LINK6_IK_J1_Y_LOCKED_JOINTS = (0, 5)
LINK6_IK_MANUAL_J5_ACTIVE_JOINTS = (0, 1, 2, 3, 5)
LINK6_IK_MANUAL_J5_LOCKED_JOINTS = (4,)
# VR uses the training-time locked wrist. Its lateral hand command is a pure
# base-yaw request: J1 owns Y, J2/J3 own the radial X/Z task, and J4/J5/J6
# remain at the training Home.
LINK6_IK_VR_ACTIVE_JOINTS = (1, 2)
LINK6_IK_VR_LOCKED_JOINTS = (0, 3, 4, 5)
LINK6_IK_REACH_SEARCH_STEPS = 8
GRIPPER_MAX_OPENING_M = 0.080
GRIPPER_EFFORT = 3000
J2_NEAR_ZERO_WRAP_MARGIN_RAD = math.radians(15.0)
_j2_wrap_reported = False


class ManualZeroRequested(Exception):
    """Raised after terminal B has safely moved Piper to joint zero."""


def ensure_can_up(can_port: str) -> None:
    result = subprocess.run(
        ["ip", "-details", "link", "show", "dev", can_port],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    if result.returncode == 0 and "state UP" in result.stdout:
        return
    helper = os.environ.get("PIPER_CAN_UP", "/home/user/piper/bin/can_up.sh")
    subprocess.run([helper, can_port], check=True)


def discover_can_port(explicit: Optional[str]) -> str:
    if explicit:
        return explicit
    helper = os.environ.get("PIPER_FIND_CAN", "/home/user/piper/bin/find_can.sh")
    result = subprocess.run(
        [helper], check=True, stdout=subprocess.PIPE, text=True
    )
    can_port = result.stdout.strip()
    if not can_port:
        raise RuntimeError("No gs_usb Piper USB-CAN interface was found")
    return can_port


def joint_message_is_fresh(message: object) -> bool:
    return (message.Hz > 0 and
            0 <= time.time() - message.time_stamp <= 0.5)


def status_message_is_fresh(message: object, now: float) -> bool:
    """Status validity comes from arrival time, not the SDK's windowed FPS.

    Driver status arrives more slowly than joint positions. A zero FPS
    estimate alone does not invalidate a newly received status frame. Never
    relax the existing 500 ms timestamp limit or accept an uninitialized cache.
    """
    stamp = message.time_stamp
    return (math.isfinite(stamp) and stamp > 0 and
            0 <= now - stamp <= 0.5)


def status_feedback_error(status: object, drivers: object, now: float) -> str:
    details = []
    for label, message in (("controller 0x2A1", status), ("motors 0x261-266", drivers)):
        if not status_message_is_fresh(message, now):
            age_ms = (now - message.time_stamp) * 1000.0
            details.append(f"{label}: age={age_ms:.1f}ms, Hz={message.Hz:.1f}")
    return "Controller/motor status feedback is stale (" + "; ".join(details) + ")"


def raw_joint_message_radians(message: object) -> List[float]:
    """Diagnostics only: never authorize motion with these unvalidated values."""
    state = message.joint_state
    scale = math.pi / 180000.0
    return [
        state.joint_1 * scale,
        state.joint_2 * scale,
        state.joint_3 * scale,
        state.joint_4 * scale,
        state.joint_5 * scale,
        state.joint_6 * scale,
    ]


def joint_message_radians(message: object) -> List[float]:
    """Decode and validate the SDK's latest cached joint frame."""
    global _j2_wrap_reported
    values = raw_joint_message_radians(message)
    # Some Piper firmware sessions report the near-zero J2 position with one
    # extra negative encoder turn (observed -369.831 deg while fresh at
    # 200 Hz, equivalent to -9.831 deg). Accept only this narrow, unambiguous
    # near-zero wrap. Do not normalize arbitrary multi-turn faults such as the
    # previously observed -267 deg sample (which would become +93 deg).
    if not FEEDBACK_SANITY_LIMITS_RAD[1][0] <= values[1] <= \
            FEEDBACK_SANITY_LIMITS_RAD[1][1]:
        wrapped_j2 = math.remainder(values[1], 2.0 * math.pi)
        if abs(wrapped_j2) <= J2_NEAR_ZERO_WRAP_MARGIN_RAD:
            if not _j2_wrap_reported:
                print(
                    "♻️ Piper J2 full-turn feedback normalized: "
                    f"{math.degrees(values[1]):.3f}° -> "
                    f"{math.degrees(wrapped_j2):.3f}°",
                    flush=True,
                )
                _j2_wrap_reported = True
            values[1] = wrapped_j2
    validate_feedback_sanity(values)
    return values


def read_joint_radians(piper: C_PiperInterface_V2) -> List[float]:
    message = piper.GetArmJointMsgs()
    if not joint_message_is_fresh(message):
        raise RuntimeError("Piper six-joint feedback has not updated for over 500 ms")
    return joint_message_radians(message)


def read_cached_joint_radians(piper: C_PiperInterface_V2) -> List[float]:
    """Return the last sane SDK sample even while live feedback is paused."""
    message = piper.GetArmJointMsgs()
    if message.Hz <= 0:
        raise RuntimeError("No valid cached Piper six-joint feedback is available")
    return joint_message_radians(message)


def validate_feedback_sanity(values: List[float]) -> None:
    for index, (value, limits) in enumerate(
            zip(values, FEEDBACK_SANITY_LIMITS_RAD), 1):
        if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
            raise RuntimeError(
                f"Invalid Piper J{index} feedback: {math.degrees(value):.2f} deg; "
                "enable/homing rejected. Check encoder zero and CAN feedback."
            )


def read_end_pose(piper: C_PiperInterface_V2) -> List[float]:
    """Return controller end pose as XYZ millimetres and RPY degrees."""
    message = piper.GetArmEndPoseMsgs()
    if message.Hz <= 0 or not 0 <= time.time() - message.time_stamp <= 0.5:
        raise RuntimeError("Piper end-pose feedback has not updated for over 500 ms")
    pose = message.end_pose
    values = [
        pose.X_axis / 1000.0,
        pose.Y_axis / 1000.0,
        pose.Z_axis / 1000.0,
        pose.RX_axis / 1000.0,
        pose.RY_axis / 1000.0,
        pose.RZ_axis / 1000.0,
    ]
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Piper end-pose feedback contains a non-finite value")
    return values


def wait_for_feedback(piper: C_PiperInterface_V2, timeout: float = 8.0) -> List[float]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = piper.GetArmJointMsgs()
        if msg.Hz > 0 and time.time() - msg.time_stamp < 1.0:
            q = read_joint_radians(piper)
            if all(math.isfinite(value) for value in q):
                return q
        time.sleep(0.05)
    raise RuntimeError("No Piper six-joint feedback was received within 8 seconds")


def wait_for_stable_feedback(
    piper: C_PiperInterface_V2,
    timeout: float = 8.0,
    stable_samples: int = 8,
    max_sample_step_deg: float = 5.0,
) -> List[float]:
    """Require consecutive sane samples before any command is enabled."""
    deadline = time.monotonic() + timeout
    previous: Optional[List[float]] = None
    stable = 0
    last_error = "no feedback received yet"
    max_step = math.radians(max_sample_step_deg)
    while time.monotonic() < deadline:
        try:
            current = read_joint_radians(piper)
            validate_feedback_sanity(current)
            if previous is not None:
                jump = max(abs(a - b) for a, b in zip(current, previous))
                if jump <= max_step:
                    stable += 1
                else:
                    stable = 0
                    last_error = (
                        f"adjacent feedback jump {math.degrees(jump):.2f} deg "
                        f"> {max_sample_step_deg:.1f} deg"
                    )
            previous = current
            if stable >= stable_samples - 1:
                return current
        except RuntimeError as exc:
            previous = None
            stable = 0
            last_error = str(exc)
        time.sleep(0.05)
    raise RuntimeError(
        f"Piper feedback was not stable for {stable_samples} consecutive frames: {last_error}"
    )


def validate_target(target: object) -> List[float]:
    if not isinstance(target, list) or len(target) != 6:
        raise ValueError("target must contain six values in radians")
    values = [float(value) for value in target]
    for index, (value, limits) in enumerate(zip(values, JOINT_LIMITS_RAD), 1):
        if not math.isfinite(value):
            raise ValueError(f"J{index} is not finite")
        if not limits[0] <= value <= limits[1]:
            raise ValueError(
                f"J{index}={math.degrees(value):.2f} deg is outside "
                f"[{math.degrees(limits[0]):.1f}, {math.degrees(limits[1]):.1f}]°"
            )
    return values


def validate_pose_offset(command: dict) -> tuple[List[float], float]:
    xyz = command.get("xyz_m")
    if not isinstance(xyz, list) or len(xyz) != 3:
        raise ValueError("pose_offset.xyz_m must contain three values in metres")
    xyz_values = [float(value) for value in xyz]
    pitch_rad = float(command.get("pitch_rad", 0.0))
    if not all(math.isfinite(value) for value in xyz_values + [pitch_rad]):
        raise ValueError("pose_offset contains a non-finite value")
    return xyz_values, pitch_rad


def validate_link6_offset(command: dict) -> List[float]:
    xyz = command.get("xyz_m")
    if not isinstance(xyz, list) or len(xyz) != 3:
        raise ValueError("link6_offset.xyz_m must contain three values in metres")
    values = [float(value) for value in xyz]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("link6_offset contains a non-finite value")
    profile = command.get("workspace_profile")
    if profile is not None:
        bounds = LINK6_WORKSPACE_OFFSET_BOUNDS_M.get(profile)
        if bounds is None:
            raise ValueError(f"Unsupported Piper workspace profile: {profile!r}")
        values = [
            max(lower, min(upper, value))
            for value, (lower, upper) in zip(values, bounds)
        ]
    return values


def validate_optional_joint5_target(command: dict) -> Optional[float]:
    if "joint5_target_rad" not in command:
        return None
    value = float(command["joint5_target_rad"])
    lower, upper = JOINT_LIMITS_RAD[4]
    if not math.isfinite(value):
        raise ValueError("link6_offset.joint5_target_rad is not finite")
    if not lower <= value <= upper:
        raise ValueError(
            "link6_offset.joint5_target_rad="
            f"{math.degrees(value):.2f} deg is outside "
            f"[{math.degrees(lower):.1f}, {math.degrees(upper):.1f}]°"
        )
    return value


def validate_optional_joint6_target(command: dict) -> Optional[float]:
    if "joint6_target_rad" not in command:
        return None
    value = float(command["joint6_target_rad"])
    lower, upper = JOINT_LIMITS_RAD[5]
    if not math.isfinite(value):
        raise ValueError("link6_offset.joint6_target_rad is not finite")
    if not lower <= value <= upper:
        raise ValueError(
            "link6_offset.joint6_target_rad="
            f"{math.degrees(value):.2f} deg is outside "
            f"[{math.degrees(lower):.1f}, {math.degrees(upper):.1f}]°"
        )
    return value


def link6_xyz_m(
    fk: C_PiperForwardKinematics, joints_rad: List[float]
) -> np.ndarray:
    pose = fk.CalFK([float(value) for value in joints_rad])[-1]
    xyz = np.asarray(pose[:3], dtype=float) * 0.001
    if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
        raise RuntimeError("Piper link6 FK returned an invalid position")
    return xyz


def solve_link6_position_ik(
    fk: C_PiperForwardKinematics,
    seed_rad: List[float],
    target_xyz_m: List[float],
    active_joints=LINK6_IK_ACTIVE_JOINTS,
    locked_joints=LINK6_IK_LOCKED_JOINTS,
    locked_joint_values: Optional[dict] = None,
) -> List[float]:
    """Continuous damped-least-squares IK for the link6 pen position.

    The previous 50 Hz solution is used as the seed.  ``active_joints`` and
    ``locked_joints`` select the VR locked-wrist or keyboard full-joint mode.
    """
    q = np.asarray(seed_rad, dtype=float).copy()
    target = np.asarray(target_xyz_m, dtype=float)
    home = np.asarray(TRAINING_HOME_JOINTS, dtype=float)
    lower = np.asarray([limit[0] for limit in JOINT_LIMITS_RAD], dtype=float)
    upper = np.asarray([limit[1] for limit in JOINT_LIMITS_RAD], dtype=float)
    if q.shape != (6,) or target.shape != (3,):
        raise ValueError("joint6 IK input has the wrong dimensions")
    locked_targets = {
        joint: float(home[joint]) for joint in locked_joints
    }
    if locked_joint_values is not None:
        locked_targets.update({
            int(joint): float(value)
            for joint, value in locked_joint_values.items()
        })
    for joint in locked_joints:
        q[joint] = locked_targets[joint]

    for _ in range(LINK6_IK_MAX_ITERATIONS):
        position = link6_xyz_m(fk, q.tolist())
        error = target - position
        if float(np.linalg.norm(error)) <= LINK6_IK_TOLERANCE_M:
            return q.tolist()

        jacobian = np.zeros((3, len(active_joints)), dtype=float)
        for column, joint in enumerate(active_joints):
            perturbed = q.copy()
            perturbed[joint] += LINK6_IK_FD_STEP_RAD
            jacobian[:, column] = (
                link6_xyz_m(fk, perturbed.tolist()) - position
            ) / LINK6_IK_FD_STEP_RAD

        gram = jacobian @ jacobian.T
        regularized = gram + (LINK6_IK_DAMPING ** 2) * np.eye(3)
        try:
            task_inverse = np.linalg.solve(regularized, np.eye(3))
        except np.linalg.LinAlgError as exc:
            raise RuntimeError("Piper joint6 IK matrix is singular") from exc
        pseudo_inverse = jacobian.T @ task_inverse
        active_q = q[list(active_joints)]
        active_home = home[list(active_joints)]
        null_space = np.eye(len(active_joints)) - pseudo_inverse @ jacobian
        delta = pseudo_inverse @ error
        delta += null_space @ (0.04 * (active_home - active_q))
        q[list(active_joints)] += np.clip(delta, -0.05, 0.05)
        q = np.clip(q, lower + 1.0e-4, upper - 1.0e-4)
        for joint in locked_joints:
            q[joint] = locked_targets[joint]

    final_error = float(np.linalg.norm(
        target - link6_xyz_m(fk, q.tolist())
    ))
    if final_error > 0.004:
        raise RuntimeError(
            f"Piper joint6 IK did not converge: tool-tip error {final_error * 1000.0:.1f} mm"
        )
    return q.tolist()


def solve_link6_position_ik_reachable(
    fk: C_PiperForwardKinematics,
    seed_rad: List[float],
    requested_xyz_m: List[float],
    active_joints=LINK6_IK_ACTIVE_JOINTS,
    locked_joints=LINK6_IK_LOCKED_JOINTS,
    locked_joint_values: Optional[dict] = None,
) -> tuple[List[float], float, List[float]]:
    """Solve the full target or the furthest reachable point toward it.

    The Cartesian workspace is nonlinear and cannot be represented well by
    one small axis-aligned box/ellipsoid.  First try the
    requested point.  If it is not solvable, bisect the segment from the last
    valid FK position toward it and return the furthest solvable point.  This
    opens the selected physical joint workspace without turning an unreachable hand
    target into a bridge exception or arm disable.
    """
    requested = np.asarray(requested_xyz_m, dtype=float)
    start = link6_xyz_m(fk, seed_rad)
    try:
        target = solve_link6_position_ik(
            fk, seed_rad, requested.tolist(), active_joints, locked_joints,
            locked_joint_values)
        return target, 1.0, requested.tolist()
    except RuntimeError:
        pass

    lower_fraction = 0.0
    upper_fraction = 1.0
    best_target = list(seed_rad)
    best_xyz = start.copy()
    for _ in range(LINK6_IK_REACH_SEARCH_STEPS):
        fraction = 0.5 * (lower_fraction + upper_fraction)
        candidate_xyz = start + fraction * (requested - start)
        try:
            candidate_target = solve_link6_position_ik(
                fk, seed_rad, candidate_xyz.tolist(),
                active_joints, locked_joints, locked_joint_values
            )
            lower_fraction = fraction
            best_target = candidate_target
            best_xyz = candidate_xyz
        except RuntimeError:
            upper_fraction = fraction
    return best_target, lower_fraction, best_xyz.tolist()


def solve_training_j123_ik(fk, seed_rad, requested_xyz_m):
    """Training wrist is fixed even when an unreachable target projects to seed.

    Normal motion starts after Z has reached Home. The caller's existing
    common slew limiter is retained for any off-Home reconnect/transition;
    this helper never bypasses that limiter or commands hardware.
    """
    seed = list(seed_rad)
    seed[3:] = TRAINING_HOME_JOINTS[3:]
    return solve_link6_position_ik_reachable(
        fk, seed, requested_xyz_m,
        LINK6_IK_ACTIVE_JOINTS, LINK6_IK_LOCKED_JOINTS,
        {joint: TRAINING_HOME_JOINTS[joint]
         for joint in LINK6_IK_LOCKED_JOINTS},
    )


def solve_training_j15_ik(fk, seed_rad, requested_xyz_m):
    """Five-joint XYZ IK: J6 is fixed even in unreachable projection fallback."""
    seed = list(seed_rad)
    seed[5] = TRAINING_HOME_JOINTS[5]
    return solve_link6_position_ik_reachable(
        fk, seed, requested_xyz_m,
        LINK6_IK_WRIST_YAW_ACTIVE_JOINTS,
        LINK6_IK_WRIST_YAW_LOCKED_JOINTS,
        {5: TRAINING_HOME_JOINTS[5]},
    )


def validate_gripper_opening(command: dict) -> float:
    opening_m = float(command.get("opening_m"))
    if not math.isfinite(opening_m):
        raise ValueError("gripper.opening_m is not finite")
    if not 0.0 <= opening_m <= GRIPPER_MAX_OPENING_M:
        raise ValueError(
            f"gripper.opening_m={opening_m:.4f} is outside "
            f"[0,{GRIPPER_MAX_OPENING_M:.3f}] m"
        )
    return opening_m


class NonblockingLineWriter:
    """Preserve JSON-line framing without making TCP backpressure fatal.

    Control replies are retained in order.  Periodic state frames are
    coalesced: while an older state has not started sending, it is replaced by
    the newest one.  If a state frame was partially sent, it is completed
    before at most one newer state is queued.  EAGAIN therefore never breaks a
    JSON line and never disables Piper merely because the controller is briefly
    slow to read telemetry.
    """

    def __init__(self) -> None:
        self._frames = []
        self._offset = 0

    @property
    def pending(self) -> bool:
        return bool(self._frames)

    def queue(self, payload: dict, *, state: bool = False) -> None:
        frame = (json.dumps(payload, separators=(",", ":")) + "\n").encode()
        kind = "state" if state else "control"
        if state:
            for index in range(len(self._frames) - 1, -1, -1):
                if self._frames[index][0] != "state":
                    continue
                # Never replace bytes from a JSON line that already started.
                if index != 0 or self._offset == 0:
                    self._frames[index] = (kind, frame)
                    return
                break
        self._frames.append((kind, frame))

    def flush(self, conn: socket.socket) -> bool:
        """Send as much as possible; return False for ordinary EAGAIN."""
        while self._frames:
            _, frame = self._frames[0]
            try:
                sent = conn.send(memoryview(frame)[self._offset:])
            except (BlockingIOError, InterruptedError):
                return False
            if sent == 0:
                raise BrokenPipeError("TCP peer closed while sending")
            self._offset += sent
            if self._offset == len(frame):
                self._frames.pop(0)
                self._offset = 0
        return True

    def flush_until_empty(self, conn: socket.socket, timeout: float) -> None:
        """Bounded flush for the final control acknowledgement."""
        deadline = time.monotonic() + timeout
        while not self.flush(conn):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("TCP control reply timed out")
            _, writable, _ = select.select([], [conn], [], remaining)
            if not writable:
                raise TimeoutError("TCP control reply timed out")


def disable_arm(piper: C_PiperInterface_V2, reason: str) -> None:
    try:
        piper.DisablePiper()
    finally:
        print(f"⚠️ Piper disabled: {reason}", flush=True)


def enable_arm(piper: C_PiperInterface_V2, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if piper.EnablePiper():
            return
        time.sleep(0.05)
    raise RuntimeError(f"Piper failed to enable within {timeout:.1f} seconds")


class ArmRecovery:
    """Nonblocking recovery, armed only by current motion requests.

    Never clears emergency-stop/error bits. CAN reconnect is receive-only;
    enabling requires fresh status, four distinct stable joint frames and
    an in-limits measured hold before any renewed trajectory target.
    """

    def __init__(self, piper):
        self.piper = piper
        self.can_worker = None
        self.last_reopen = -math.inf
        self.reason = "waiting for stable feedback"
        self.reason_log_time = -math.inf
        self.last_enable = -math.inf
        self.reset_session()

    def reset_session(self):
        self.inhibited = False
        self.previous = None
        self.last_stamp = None
        self.stable = 0
        self.need_seed = True
        self.seed = None
        self.ready = False
        self.physical_enabled = False

    def suspend(self):
        self.inhibited = True
        self.ready = False
        self.reason = "manual exit/disable; automatic recovery is inhibited"

    def _wait(self, reason):
        self.ready = False
        self.reason = reason

    @staticmethod
    def _code(value):
        return int(getattr(value, "value", value))

    def _reopen_if_stale(self):
        now = time.monotonic()
        try:
            message = self.piper.GetArmJointMsgs()
            if time.time() - message.time_stamp < 1.0:
                return
        except Exception:
            # A missing/failed receive cache is also a reason to retry CAN;
            # it must not permanently disable receive-only reconnection.
            pass
        if (now - self.last_reopen < 2.0 or
                (self.can_worker is not None and self.can_worker.is_alive())):
            return
        self.last_reopen = now

        def reopen():
            try:
                self.piper.DisconnectPort(thread_timeout=0.1)
                time.sleep(0.15)  # Let the old SDK reader observe socket closure.
                self.piper.ConnectPort(can_init=True, piper_init=False)
            except Exception as exc:
                print(f"⚠️ Piper CAN reconnection is incomplete: {exc}; retrying later.", flush=True)

        self.previous = None
        self.last_stamp = None
        self.stable = 0
        self.need_seed = True
        self.can_worker = threading.Thread(target=reopen, daemon=True)
        self.can_worker.start()

    def observe(self):
        try:
            if self.can_worker is not None and self.can_worker.is_alive():
                raise RuntimeError("CAN receive connection is being rebuilt")
            q = read_joint_radians(self.piper)
            validate_feedback_sanity(q)
            stamp = self.piper.GetArmJointMsgs().time_stamp
            status = self.piper.GetArmStatus()
            drivers = self.piper.GetArmLowSpdInfoMsgs()
            status_now = time.time()
            if (not status_message_is_fresh(status, status_now) or
                    not status_message_is_fresh(drivers, status_now)):
                raise RuntimeError(status_feedback_error(status, drivers, status_now))
            arm_status = status.arm_status
            if (self._code(arm_status.ctrl_mode) not in (0, 1) or
                    self._code(arm_status.arm_status) not in (0, 6) or
                    self._code(arm_status.err_code) != 0):
                raise RuntimeError(
                    f"Fault or non-CAN mode: ctrl={arm_status.ctrl_mode}, "
                    f"status={arm_status.arm_status}, err={arm_status.err_code}; faults are not cleared automatically")
            enabled = []
            faults = ("voltage_too_low", "motor_overheating", "driver_overcurrent",
                      "driver_overheating", "collision_status",
                      "driver_error_status", "stall_status")
            for index in range(1, 7):
                motor = getattr(drivers, f"motor_{index}")
                if motor.can_id != 0x260 + index:
                    raise RuntimeError(f"J{index} motor status has not been received")
                flags = motor.foc_status
                if any(getattr(flags, field) for field in faults):
                    raise RuntimeError(f"J{index} motor fault/protection remains active; faults are not cleared automatically")
                enabled.append(bool(flags.driver_enable_status))
            self.physical_enabled = (all(enabled) and
                                     self._code(arm_status.arm_status) == 0)
            if stamp != self.last_stamp:
                if (self.previous is None or max(abs(a-b) for a,b in
                        zip(q, self.previous)) <= math.radians(5.0)):
                    self.stable += 1
                else:
                    self.stable = 1
                self.previous = list(q)
                self.last_stamp = stamp
            if not self.physical_enabled:
                self.need_seed = True
                self.ready = False
            return q
        except (RuntimeError, OSError, AttributeError, TypeError, ValueError) as exc:
            self.previous = None
            self.last_stamp = None
            self.stable = 0
            self.need_seed = True
            self.physical_enabled = False
            self._wait(str(exc))
            self._reopen_if_stale()
            return None

    def prepare(self, startup=False):
        """Return (ready, measured_reseed_or_None); called on motion intent."""
        if self.inhibited:
            return False, None
        q = self.observe()
        if q is None or self.stable < 4:
            self._wait(self.reason if q is None else "waiting for four consecutive stable feedback frames")
            return False, None
        if self.need_seed:
            limits = STARTUP_HOLD_LIMITS_RAD if startup else JOINT_LIMITS_RAD
            hold = [max(lo, min(hi, value)) for value, (lo, hi) in zip(q, limits)]
            # Mechanical-zero startup retains its existing calibrated-margin
            # clamp. Policy recovery never broadens its measured-hold limits.
            if not startup and max(abs(a-b) for a,b in zip(q, hold)) > math.radians(1.0):
                self._wait("measured pose is outside the joint command range; manual inspection required")
                return False, None
            if time.monotonic() - self.last_enable < 0.2:
                return False, None
            self.last_enable = time.monotonic()
            try:
                send_move_j(self.piper, hold, 15)
                if not self.physical_enabled:
                    self.piper.EnablePiper()  # Confirmation comes from later status.
                    self._wait("measured pose latched and enable requested; waiting for all six motors")
                    return False, None
            except (RuntimeError, OSError) as exc:
                self._wait(f"recovery command was not completed: {exc}")
                return False, None
            self.seed = list(hold)
            self.need_seed = False
            return False, None  # One measured hold precedes renewed motion.
        seed, self.seed = self.seed, None
        self.ready = True
        self.reason = "normal"
        return True, seed

    def wait_feedback(self):
        """For startup/homing: pause on gaps, without sending blind targets."""
        while True:
            q = self.observe()
            if q is not None and self.stable >= 4:
                return q
            time.sleep(0.02)


class StartupZero:
    """Single-threaded staged startup motion; TCP polling never blocks on it."""

    def __init__(self, piper, recovery, speed, timeout, tolerance_deg,
                 delay, initialize_gripper=True):
        self.piper, self.recovery = piper, recovery
        self.speed, self.timeout = speed, timeout
        self.tolerance = math.radians(tolerance_deg)
        self.start_after = time.monotonic() + delay
        self.initialize_gripper = initialize_gripper
        self.active, self.complete = True, False
        self.phase = 0
        self.goal = self.sent = None
        self.last_tick = None
        self.last_send = -math.inf
        self.elapsed = 0.0
        self.settled = 0
        self.stamp = None

    def cancel(self):
        self.active = False

    def step(self):
        if not self.active or time.monotonic() < self.start_after:
            return
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), .1) if self.last_tick is not None else 0.
        self.last_tick = now
        q = self.recovery.observe()
        if q is None or self.recovery.inhibited:
            self.settled = 0
            return
        ready, seed = self.recovery.prepare(startup=True)
        if not ready:
            self.settled = 0
            return
        if seed is not None:
            self.sent = list(seed)
        if self.phase == 0:
            self.goal = list(self.sent)
            self.goal[:2] = [0., 0.]
            self.phase = 1
        self.elapsed += dt
        if self.elapsed > self.timeout:
            self.cancel()
            self.recovery.suspend()
            try:
                disable_arm(self.piper, "startup homing timed out; communication continues and manual inspection is required")
            except (RuntimeError, OSError) as exc:
                print(f"⚠️ Startup-timeout disable was not confirmed: {exc}; TCP remains available.", flush=True)
            print("⚠️ Startup homing did not complete; automatic motion stopped and TCP remains available.", flush=True)
            return
        if now - self.last_send >= .02:
            step = .8 * min(max(now - self.last_send, .001), .05)
            self.sent = [old + max(-step, min(step, goal-old))
                         for old, goal in zip(self.sent, self.goal)]
            try:
                send_move_j(self.piper, self.sent, self.speed)
            except (RuntimeError, OSError) as exc:
                self.recovery.need_seed = True
                self.recovery._wait(f"startup homing command is waiting: {exc}")
                self.settled = 0
                return
            self.last_send = now
        stamp = self.recovery.last_stamp
        if stamp != self.stamp:
            self.stamp = stamp
            checked = (0, 1) if self.phase == 1 else range(6)
            reached = all(abs(q[i]-self.goal[i]) <= self.tolerance for i in checked)
            self.settled = self.settled + 1 if reached else 0
        if self.settled >= 5:
            if self.phase == 1:
                self.phase, self.goal, self.settled, self.elapsed = 2, [0.]*6, 0, 0.
            else:
                self.active, self.complete = False, True
                self.recovery.need_seed = True
                print("✅ Piper homing complete; all six joints are at zero and ready for policy control.", flush=True)
                if self.initialize_gripper:
                    try:
                        close_gripper(self.piper)
                    except (RuntimeError, OSError) as exc:
                        print(f"⚠️ Homing completed, but gripper initialization failed: {exc}; TCP remains available.", flush=True)


class PolicyPoseRecovery:
    """Restore the last actually sent joint target, not queued IK requests.

    Authorized only by earlier motion in this TCP session. Explicit exit,
    disable and a healthy-link watchdog cancel it. Faults and stale feedback
    never authorize a measured hold or re-enable. No queued new command is
    replayed during restoration; the live stream resumes after convergence.
    """

    def __init__(self, piper, recovery, speed, max_step):
        self.piper, self.recovery = piper, recovery
        self.speed, self.max_step = speed, max_step
        self.goal = self.sent = None
        self.pending = False
        self.last_send = -math.inf
        self.last_tick = None
        self.elapsed = 0.
        self.settled, self.stamp = 0, None

    def remember(self, target):
        self.goal = list(validate_target(list(target)))

    def cancel(self):
        self.goal = self.sent = None
        self.pending = False

    def interrupt(self):
        if self.goal is not None and not self.pending and not self.recovery.inhibited:
            self.pending = True
            self.recovery.need_seed = True
            self.sent = None
            self.last_tick = None
            self.elapsed, self.settled, self.stamp = 0., 0, None
            print("🔄 Unexpected Piper disable or feedback loss: the last joint target was saved; waiting for safe re-enable and slew-limited recovery.", flush=True)

    def step(self):
        if not self.pending or self.recovery.inhibited:
            return None
        now = time.monotonic()
        dt = min(max(now-self.last_tick, 0.), .1) if self.last_tick is not None else 0.
        self.last_tick = now
        ready, seed = self.recovery.prepare()
        if not ready:
            self.settled = 0
            return None
        if seed is not None:
            self.sent = list(seed)
            self.last_send = now
        if self.sent is None:
            self.sent = list(self.recovery.previous)
            self.last_send = now
        self.elapsed += dt
        if self.elapsed > 30.:
            self.cancel()
            self.recovery.suspend()
            try:
                disable_arm(self.piper, "pre-disable pose recovery timed out; communication remains available and manual inspection is required")
            except (RuntimeError, OSError) as exc:
                print(f"⚠️ Recovery-timeout disable was not confirmed: {exc}; TCP remains available.", flush=True)
            return None
        if now-self.last_send >= .02:
            step = min(self.max_step, .8*min(now-self.last_send, .05))
            target = [old+max(-step, min(step, goal-old))
                      for old,goal in zip(self.sent,self.goal)]
            try:
                send_move_j(self.piper, target, self.speed)
            except (RuntimeError, OSError) as exc:
                self.recovery.need_seed = True
                self.recovery._wait(f"policy-pose recovery command is waiting: {exc}")
                return None
            self.sent, self.last_send = target, now
        q = self.recovery.previous
        if self.recovery.last_stamp != self.stamp:
            self.stamp = self.recovery.last_stamp
            reached = (max(abs(a-b) for a,b in zip(q,self.goal)) <= math.radians(1.)
                       and max(abs(a-b) for a,b in zip(self.sent,self.goal)) < 1.e-6)
            self.settled = self.settled+1 if reached else 0
        if self.settled >= 4:
            self.pending = False
            print("✅ Piper re-enabled and restored the pre-fault joint target; real-time policy control resumed without homing.", flush=True)
            return list(self.goal)
        return None


def close_gripper(piper: C_PiperInterface_V2) -> None:
    """Clear gripper errors, enable it, and latch the fully closed target."""
    closed_units = 0
    piper.GripperCtrl(closed_units, GRIPPER_EFFORT, 0x03, 0)
    time.sleep(0.05)
    for _ in range(10):
        piper.GripperCtrl(closed_units, GRIPPER_EFFORT, 0x01, 0)
        time.sleep(0.02)
    print("🗜️ Piper gripper initialized: fully closed.", flush=True)


def send_move_j(piper: C_PiperInterface_V2, target: List[float],
                speed_percent: int) -> None:
    """Send one Piper MOVE_J target expressed in radians."""
    piper.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
    piper.JointCtrl(*(round(value * RAD_TO_MILLI_DEG) for value in target))


def send_move_p(piper: C_PiperInterface_V2, target_pose: List[float],
                speed_percent: int) -> None:
    """Send XYZ(mm), RPY(deg) through Piper's official MOVE_P IK."""
    piper.MotionCtrl_2(0x01, 0x00, speed_percent, 0x00)
    scaled = [round(value * 1000.0) for value in target_pose]
    piper.EndPoseCtrl(*scaled)


def hold_current_joint_pose(
    piper: C_PiperInterface_V2,
    speed_percent: int,
    duration: float = 0.5,
) -> List[float]:
    """Latch the measured six-joint pose before starting a shutdown move."""
    target = wait_for_stable_feedback(piper)
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        send_move_j(piper, target, speed_percent)
        time.sleep(0.05)
    return target


def move_to_joint_zero(
    piper: C_PiperInterface_V2,
    speed_percent: int,
    timeout: float,
    tolerance_deg: float,
    stable_cycles: int = 5,
    recovery: Optional[ArmRecovery] = None,
) -> List[float]:
    """Enable at the measured pose, then perform staged mechanical homing.

    Feedback can be a few degrees outside the nominal software limits after
    calibration.  The initial hold command is therefore clamped to the valid
    range before enabling.  Every mechanical-zero request uses two stages:
    stage 1 holds J3-J6 at the captured target and moves J1/J2 to zero;
    stage 2 starts only after J1/J2 have settled and moves J3-J6 to zero.
    """
    current = (wait_for_stable_feedback(piper) if recovery is None
               else recovery.wait_feedback())
    hold_target = [
        max(lower, min(upper, value))
        for value, (lower, upper) in zip(current, STARTUP_HOLD_LIMITS_RAD)
    ]
    zero_target = [0.0] * 6

    print("Enabling Piper MOVE_J slowly at the measured pose...", flush=True)
    enabled = False
    try:
        enable_deadline = time.monotonic() + 5.0
        while recovery is not None or time.monotonic() < enable_deadline:
            if recovery is not None:
                current = recovery.wait_feedback()
                hold_target = [max(lo, min(hi, value)) for value, (lo, hi)
                               in zip(current, STARTUP_HOLD_LIMITS_RAD)]
                if time.monotonic() >= enable_deadline:
                    recovery._wait("startup enable is not confirmed; keeping the service available while waiting for stable feedback")
                    enable_deadline = time.monotonic() + 5.0
            send_move_j(piper, hold_target, speed_percent)
            if piper.EnablePiper():
                enabled = True
                break
            time.sleep(0.05)
        if not enabled:
            raise RuntimeError("Piper failed to enable within 5.0 seconds")

        tolerance_rad = math.radians(tolerance_deg)

        def move_phase(
            target: List[float],
            checked_joints: tuple[int, ...],
            phase_name: str,
            completion_text: str,
        ) -> List[float]:
            nonlocal current
            deadline = time.monotonic() + timeout
            stable = 0
            last_report = 0.0
            previous_feedback = current
            while time.monotonic() < deadline:
                if recovery is not None:
                    pause_start = time.monotonic()
                    before = recovery.observe()
                    if before is None or not recovery.physical_enabled:
                        before = recovery.wait_feedback()
                        if not recovery.physical_enabled:
                            # A real driver disable needs a confirmed measured
                            # hold, never a replay of the homing endpoint.
                            was_inhibited = recovery.inhibited
                            recovery.inhibited = False
                            try:
                                while not recovery.prepare()[0]:
                                    time.sleep(0.02)
                            finally:
                                recovery.inhibited = was_inhibited
                        deadline += time.monotonic() - pause_start
                        previous_feedback = before
                        stable = 0
                send_move_j(piper, target, speed_percent)
                time.sleep(0.05)
                try:
                    current = read_joint_radians(piper)
                except RuntimeError:
                    if recovery is None:
                        raise
                    pause_start = time.monotonic()
                    current = recovery.wait_feedback()
                    deadline += time.monotonic() - pause_start
                    previous_feedback = current
                    stable = 0
                    continue
                validate_feedback_sanity(current)
                feedback_jump = max(
                    abs(a - b) for a, b in zip(current, previous_feedback)
                )
                if feedback_jump > math.radians(45.0):
                    raise RuntimeError(
                        f"Sudden joint-feedback jump during {phase_name}: "
                        f"{math.degrees(feedback_jump):.2f} deg; stopping immediately"
                    )
                previous_feedback = current

                max_error = max(
                    abs(current[index] - target[index])
                    for index in checked_joints
                )
                stable = stable + 1 if max_error <= tolerance_rad else 0
                if stable >= stable_cycles:
                    print("\n" + completion_text + ": " + " ".join(
                        f"J{i + 1}={math.degrees(value):.2f}°"
                        for i, value in enumerate(current)
                    ), flush=True)
                    return current

                now = time.monotonic()
                if now - last_report >= 1.0:
                    print("\r" + phase_name + ": " + " ".join(
                        f"J{i + 1}={math.degrees(value):6.2f}°"
                        for i, value in enumerate(current)
                    ), end="", flush=True)
                    last_report = now
            raise RuntimeError(
                f"Piper did not complete {phase_name} within {timeout:.1f} seconds"
            )

        joints12_zero_target = hold_target.copy()
        joints12_zero_target[0] = 0.0
        joints12_zero_target[1] = 0.0
        print(
            f"✅ Piper enabled; stage 1 homes J1/J2 at {speed_percent}% speed "
            "while holding J3-J6 at their startup targets.",
            flush=True,
        )
        move_phase(
            joints12_zero_target,
            checked_joints=(0, 1),
            phase_name="stage 1 J1/J2 homing (holding J3-J6)",
            completion_text="✅ Stage 1 complete; J1/J2 reached zero",
        )

        print(
            "Stage 2 started: J1/J2 are stable at zero; "
            f"homing J3-J6 at {speed_percent}% speed.",
            flush=True,
        )
        return move_phase(
            zero_target,
            checked_joints=(0, 1, 2, 3, 4, 5),
            phase_name="stage 2 J3-J6 homing",
            completion_text="✅ Piper all-six-joint zero position reached",
        )
    except BaseException:
        if enabled:
            disable_arm(piper, "startup homing failed or was interrupted")
        raise


def coalesce_training_xyz_commands(
    command: dict, pending: bytearray, previous_request=None
):
    """Use the latest consecutive training XYZ sample, not queued old targets.

    Only canonical training XYZ protocols are eligible. Home, gripper,
    disable, profile changes and legacy/manual-wrist commands are barriers and
    are never dropped/reordered. Every replaced XYZ is validated first.
    """
    allowed_keys = {
        "type", "xyz_m", "workspace_profile", "ik_profile",
        "allow_all_joints", "exact_training_home",
    }

    def eligible(value):
        if not isinstance(value, dict):
            return False
        if (value.get("type") == "pose_offset" and
                value.get("pose_profile") == OFFICIAL_MOVE_P_PROFILE):
            if (value.get("exact_training_home") is True or
                    not set(value).issubset({
                        "type", "xyz_m", "pitch_rad", "workspace_profile",
                        "pose_profile", "exact_training_home"})):
                return False
            xyz = validate_link6_offset(value)
            _, pitch = validate_pose_offset(value)
            return pitch == 0.0 and math.sqrt(sum(v*v for v in xyz)) > 1.0e-4
        if (value.get("type") != "link6_offset" or
                value.get("ik_profile") not in (
                    TRAINING_J123_PROFILE, TRAINING_Y_J1_PROFILE,
                    TRAINING_J15_PROFILE) or
                value.get("allow_all_joints") is not False or
                value.get("exact_training_home") is True or
                not set(value).issubset(allowed_keys)):
            return False
        xyz = validate_link6_offset(value)
        # Zero offset also requests exact Home in the legacy protocol.
        return math.sqrt(sum(v * v for v in xyz)) > 1.0e-4

    if not eligible(command):
        return command, pending
    while b"\n" in pending:
        raw, _, remainder = pending.partition(b"\n")
        try:
            candidate = json.loads(raw.decode("utf-8"))
            if (not eligible(candidate) or
                    candidate.get("ik_profile") != command.get("ik_profile") or
                    candidate.get("pose_profile") != command.get("pose_profile") or
                    candidate.get("workspace_profile") !=
                    command.get("workspace_profile")):
                break
            if command.get("ik_profile") == TRAINING_Y_J1_PROFILE:
                previous = previous_request or [0.0, 0.0, 0.0]
                current_xyz = validate_link6_offset(command)
                candidate_xyz = validate_link6_offset(candidate)
                if (abs(candidate_xyz[1] - current_xyz[1]) > 1.0e-6 and
                        any(abs(current_xyz[i] - previous[i]) > 1.0e-6
                            for i in (0, 2))):
                    # Do not merge an unprocessed X/Z move followed by Y into
                    # a mixed XYZ packet: Y must freeze the preceding target.
                    break
        except (ValueError, TypeError, UnicodeError):
            # Leave bad input queued for the normal parser/error path; never
            # silently discard malformed commands while coalescing motion.
            break
        command = candidate
        pending = bytearray(remainder)
    return command, pending


def serve_client(
    conn: socket.socket,
    piper: C_PiperInterface_V2,
    rate_hz: float,
    speed_percent: int,
    watchdog_seconds: float,
    max_step_rad: float,
    training_home_speed_percent: Optional[int] = None,
    training_home_max_step_rad: Optional[float] = None,
    keyboard_fd: Optional[int] = None,
    initially_enabled: bool = False,
    official_move_p_speed_percent: Optional[int] = None,
    link6_rate_hz: float = 1.0 / LINK6_IK_SEND_PERIOD,
    recovery: Optional[ArmRecovery] = None,
    startup_zero: Optional[StartupZero] = None,
) -> bool:
    if recovery is not None and not (startup_zero is not None and startup_zero.active):
        recovery.reset_session()
    pose_recovery = (PolicyPoseRecovery(piper, recovery, speed_percent, max_step_rad)
                     if recovery is not None else None)
    conn.setblocking(False)
    writer = NonblockingLineWriter()
    pending = bytearray()
    period = 1.0 / rate_hz
    link6_send_period = 1.0 / link6_rate_hz
    next_state = time.monotonic()
    last_command = time.monotonic()
    try:
        previous_q = read_joint_radians(piper)
        feedback_fresh = True
    except RuntimeError as exc:
        # Server startup already required stable feedback. On a later client
        # reconnect, use the last sane cached sample only to freeze state and
        # wait; no new arm motion is sent until live feedback returns.
        try:
            previous_q = read_cached_joint_radians(piper)
        except RuntimeError:
            if recovery is None:
                raise
            previous_q = list(TRAINING_HOME_JOINTS)
        feedback_fresh = False
        print(
            f"⚠️ Piper joint feedback was unavailable when the controller connected ({exc}); "
            "holding the last target, pausing new motion, and keeping TCP connected.",
            flush=True,
        )
    previous_time = time.monotonic()
    last_target: Optional[List[float]] = None
    pose_reference: Optional[List[float]] = None
    last_pose_profile = None
    last_pose_target: Optional[List[float]] = None
    last_pose_send_time = 0.0
    last_joint_send_time = 0.0
    last_joint_limit_log_time = 0.0
    last_joint_speed_percent: Optional[int] = None
    link6_fk = C_PiperForwardKinematics(dh_is_offset=1)
    last_link6_send_time = 0.0
    last_link6_limit_log_time = 0.0
    processed_link6_xyz_request: Optional[List[float]] = None
    last_link6_joint5_request: Optional[float] = None
    pending_j5_only_target: Optional[float] = None
    last_gripper_target_units: Optional[int] = None
    control_mode: Optional[str] = None
    enabled = initially_enabled
    hold_on_disconnect = False
    ik_unreachable = False
    ik_reachable_fraction = 1.0
    exact_training_home_active = False
    last_link6_profile = None
    training_y_hold_active = False
    recovery_motion_pending = False

    while True:
        now = time.monotonic()
        timeout = max(0.0, min(0.05, next_state - now))
        readers = [conn]
        if keyboard_fd is not None:
            readers.append(keyboard_fd)
        writers = [conn] if writer.pending else []
        readable, writable, _ = select.select(readers, writers, [], timeout)

        if conn in writable:
            writer.flush(conn)

        if keyboard_fd is not None and keyboard_fd in readable:
            key = os.read(keyboard_fd, 1).lower()
            if key == b"b":
                if startup_zero is not None:
                    startup_zero.cancel()
                if pose_recovery is not None:
                    pose_recovery.cancel()
                    recovery.suspend()
                recovery_motion_pending = False
                if recovery is not None and (recovery.observe() is None or recovery.stable < 4):
                    print("⚠️ B homing was not started because feedback is not yet stable; communication remains available. Press B again after recovery.", flush=True)
                    continue
                print(
                    "\n🏠 B received: stopping policy arm targets, homing J1/J2 first, "
                    "then J3-J6, and finally closing the listener.",
                    flush=True,
                )
                move_to_joint_zero(
                    piper,
                    speed_percent=15,
                    timeout=30.0,
                    tolerance_deg=1.0,
                    recovery=recovery,
                )
                raise ManualZeroRequested()

        if conn in readable:
            chunk = conn.recv(8192)
            if not chunk:
                if recovery is not None and not hold_on_disconnect:
                    if startup_zero is not None:
                        startup_zero.cancel()
                    recovery.suspend()
                if enabled or (recovery is not None and not hold_on_disconnect):
                    if hold_on_disconnect:
                        print(
                            "🧲 Controller exited cleanly: Piper is holding the last pose and waiting for the next connection.",
                            flush=True,
                        )
                    else:
                        disable_arm(piper, "controller disconnected unexpectedly")
                        enabled = False
                return enabled
            pending.extend(chunk)
            if len(pending) > 65536:
                raise ValueError("receive buffer exceeded 64 KiB")
            while b"\n" in pending:
                raw, _, remainder = pending.partition(b"\n")
                pending = bytearray(remainder)
                if not raw.strip():
                    continue
                command = json.loads(raw.decode("utf-8"))
                command, pending = coalesce_training_xyz_commands(
                    command, pending,
                    processed_link6_xyz_request if isinstance(command, dict)
                    and command.get("ik_profile") == last_link6_profile
                    else None)
                command_type = command.get("type")
                if command_type == "subscribe":
                    continue
                if command_type == "hold":
                    if startup_zero is not None:
                        startup_zero.cancel()
                    if pose_recovery is not None:
                        pose_recovery.cancel()
                        recovery.suspend()
                    recovery_motion_pending = False
                    # This is sent only for an intentional SIGINT from the VR
                    # controller.  Keep the final controller target latched;
                    # an unannounced TCP loss still follows the fail-closed
                    # path above.
                    hold_on_disconnect = enabled
                    last_command = time.monotonic()
                    print(
                        "🧲 Clean-exit hold request received; the last pose will be held after TCP closes.",
                        flush=True,
                    )
                    continue
                if command_type == "home_disable":
                    if startup_zero is not None:
                        startup_zero.cancel()
                    if pose_recovery is not None:
                        pose_recovery.cancel()
                        recovery.suspend()
                    recovery_motion_pending = False
                    if recovery is not None and (recovery.observe() is None or recovery.stable < 4):
                        writer.queue({"type": "home_disable_complete", "ok": False,
                                      "reason": "feedback_invalid", "detail": recovery.reason})
                        print("⚠️ Homing was not started because feedback is invalid; the controller was notified and TCP remains available.", flush=True)
                        continue
                    if recovery is not None:
                        recovery.suspend()
                    # An intentional Ctrl+C from the policy controller, NOT
                    # SIGINT in this Orin service terminal.
                    # Take ownership before the TCP peer exits: first latch the
                    # measured pose, then perform the two-stage low-speed zero
                    # move, and disable only after arrival.
                    hold_on_disconnect = True
                    last_command = time.monotonic()
                    print(
                        "🧲 Policy-controller Ctrl+C homing request received: holding the current pose for 0.5 seconds, "
                        "then homing J1/J2 first and J3-J6 second.",
                        flush=True,
                    )
                    hold_current_joint_pose(
                        piper, speed_percent=15, duration=0.5)
                    move_to_joint_zero(
                        piper,
                        speed_percent=15,
                        timeout=30.0,
                        tolerance_deg=1.0,
                        recovery=recovery,
                    )
                    disable_arm(piper, "Ctrl+C homing completed")
                    enabled = False
                    writer.queue({
                        "type": "home_disable_complete",
                        "ok": True,
                    })
                    writer.flush_until_empty(conn, 0.5)
                    return False
                if command_type == "disable":
                    if startup_zero is not None:
                        startup_zero.cancel()
                    if pose_recovery is not None:
                        pose_recovery.cancel()
                    if recovery is not None:
                        recovery.suspend()
                    recovery_motion_pending = False
                    if enabled or recovery is not None:
                        disable_arm(piper, "controller disable command received")
                    enabled = False
                    last_target = None
                    pose_reference = None
                    last_pose_target = None
                    last_pose_send_time = 0.0
                    last_joint_send_time = 0.0
                    last_joint_speed_percent = None
                    last_link6_send_time = 0.0
                    processed_link6_xyz_request = None
                    last_link6_joint5_request = None
                    pending_j5_only_target = None
                    control_mode = None
                    hold_on_disconnect = False
                    ik_unreachable = False
                    ik_reachable_fraction = 1.0
                    exact_training_home_active = False
                    continue
                if command_type not in (
                    "joint", "pose_offset", "link6_offset", "gripper"
                ):
                    raise ValueError(f"Unknown command type: {command_type!r}")

                # Exclusive ownership: startup zero -> saved-pose recovery ->
                # live stream. Never queue new motion during either operation.
                if ((startup_zero is not None and startup_zero.active) or
                        (pose_recovery is not None and pose_recovery.pending)):
                    last_command = time.monotonic()
                    continue

                # Cancel the old feedback-timeout disconnect/disable path.
                # Keep the watchdog and TCP session alive, but never advance
                # IK or send blind motion while joint feedback is stale.
                if not feedback_fresh:
                    last_command = time.monotonic()
                    continue

                if recovery is not None:
                    if pose_recovery.goal is not None and command_type != "gripper":
                        observed = recovery.observe()
                        if observed is None or not recovery.physical_enabled or recovery.need_seed:
                            pose_recovery.interrupt()
                            if pose_recovery.pending:
                                recovery_motion_pending = False
                                last_command = time.monotonic()
                                continue
                    if command_type == "gripper":
                        # A gripper keepalive alone cannot rearm six joints.
                        if not recovery.ready or recovery.inhibited:
                            continue
                    else:
                        # Malformed/unbounded commands never authorize rearm.
                        if command_type == "joint":
                            validate_target(command.get("target"))
                        elif command_type == "link6_offset":
                            validate_link6_offset(command)
                        elif command_type == "pose_offset":
                            validate_pose_offset(command)
                        recovery_motion_pending = True
                        ready, measured_seed = recovery.prepare()
                        if not ready:
                            enabled = False
                            last_command = time.monotonic()
                            continue
                        enabled = True
                        recovery_motion_pending = False
                        if measured_seed is not None:
                            last_target = measured_seed
                            control_mode = None
                            pose_reference = None
                            last_pose_target = None
                            last_pose_send_time = last_joint_send_time = 0.0
                            last_link6_send_time = 0.0
                            processed_link6_xyz_request = None
                            last_link6_joint5_request = None
                            pending_j5_only_target = None
                            training_y_hold_active = False

                if command_type == "gripper":
                    opening_m = validate_gripper_opening(command)
                    binary_only = command.get("binary_only") is True
                    if binary_only:
                        opening_m = (
                            GRIPPER_MAX_OPENING_M
                            if opening_m >= GRIPPER_MAX_OPENING_M * 0.5
                            else 0.0
                        )
                    # Piper SDK uses 0.001 mm units: 80 mm == 80000.
                    opening_units = round(opening_m * 1_000_000.0)
                    target_changed = opening_units != last_gripper_target_units
                    # A new operator target also clears a latched gripper
                    # driver error. Subsequent 5 Hz keepalive frames use the
                    # normal enable command and do not spam error clearing.
                    gripper_code = 0x03 if target_changed else 0x01
                    piper.GripperCtrl(
                        opening_units, GRIPPER_EFFORT, gripper_code, 0)
                    last_gripper_target_units = opening_units
                    if target_changed:
                        if binary_only:
                            state_text = (
                                "fully open" if opening_units > 0 else "fully closed"
                            )
                            print(
                                f"🗜️ Piper gripper: {state_text} "
                                "(enabled and gripper errors cleared).",
                                flush=True,
                            )
                        else:
                            print(
                                f"🗜️ Piper gripper target: "
                                f"{opening_m * 1000.0:.0f} mm "
                                "(enabled and gripper errors cleared).",
                                flush=True,
                            )
                    continue

                if command_type == "pose_offset":
                    ik_unreachable = False
                    ik_reachable_fraction = 1.0
                    hold_on_disconnect = False
                    xyz_offset_m, pitch_offset_rad = validate_pose_offset(command)
                    pose_profile = command.get("pose_profile")
                    if pose_profile not in (None, OFFICIAL_MOVE_P_PROFILE):
                        raise ValueError(f"Unknown official end-pose profile: {pose_profile!r}")
                    official_training_pose = pose_profile == OFFICIAL_MOVE_P_PROFILE
                    if official_training_pose:
                        xyz_offset_m = validate_link6_offset(command)
                        if pitch_offset_rad != 0.0:
                            raise ValueError("Official mode keeps the training-Home tool orientation")
                        if command.get("exact_training_home") is True:
                            # Exact H is a known joint target sent through the
                            # official MOVE_J/JointCtrl API, NOT an inverse solve.
                            # Cartesian firmware IK alone cannot guarantee that
                            # a redundant end pose selects the exact Home joints.
                            now = time.monotonic()
                            if (control_mode != "official_home" or
                                    now-last_joint_send_time >= JOINT_SEND_PERIOD):
                                seed = (last_target if control_mode == "official_home"
                                        and last_target is not None
                                        else read_joint_radians(piper))
                                step = (training_home_max_step_rad
                                        if training_home_max_step_rad is not None
                                        else max_step_rad)
                                target = [old + max(-step, min(step, goal-old))
                                          for old, goal in zip(seed, TRAINING_HOME_JOINTS)]
                                if not enabled:
                                    enable_arm(piper)
                                    enabled = True
                                home_speed = (training_home_speed_percent
                                              if training_home_speed_percent is not None
                                              else speed_percent)
                                send_move_j(piper, target, home_speed)
                                if pose_recovery is not None:
                                    pose_recovery.remember(target)
                                last_target = target
                                last_joint_send_time = time.monotonic()
                            if control_mode != "official_home":
                                print("🏠 H: using official MOVE_J to reach the exact training Home without custom IK.", flush=True)
                            control_mode = "official_home"
                            pose_reference = None
                            last_pose_target = None
                            last_pose_send_time = 0.0
                            last_command = time.monotonic()
                            continue
                    entering_pose = (control_mode != "pose" or pose_reference is None
                                     or last_pose_profile != pose_profile)
                    if entering_pose:
                        try:
                            current_pose = read_end_pose(piper)
                        except RuntimeError as exc:
                            if not official_training_pose:
                                raise
                            # Do not switch modes blindly or tear down TCP
                            # if official pose feedback is temporarily absent.
                            now = time.monotonic()
                            if now-last_pose_send_time >= 1.0:
                                print(f"⚠️ Official MOVE_P is waiting for valid end-pose feedback: {exc}; no new motion is sent.", flush=True)
                                last_pose_send_time = now
                            last_command = now
                            continue
                        pose_reference = (list(link6_fk.CalFK(
                            list(TRAINING_HOME_JOINTS))[-1])
                            if official_training_pose else list(MOVE_P_HOME_POSE))
                        last_pose_profile = pose_profile
                        last_pose_target = None
                        print(
                            "✅ Piper entered official MOVE_P; fixed Home reference="
                            f"XYZ({pose_reference[0]:.1f}, {pose_reference[1]:.1f}, "
                            f"{pose_reference[2]:.1f}) mm, "
                            f"RPY({pose_reference[3]:.1f}, {pose_reference[4]:.1f}, "
                            f"{pose_reference[5]:.1f}) deg; current="
                            f"XYZ({current_pose[0]:.1f}, {current_pose[1]:.1f}, "
                            f"{current_pose[2]:.1f}) mm",
                            flush=True,
                        )
                    target_pose = list(pose_reference)
                    for index in range(3):
                        target_pose[index] += xyz_offset_m[index] * 1000.0
                    target_pose[4] += math.degrees(pitch_offset_rad)

                    if not enabled:
                        enable_arm(piper)
                        enabled = True
                    # The 103 main loop is 200 Hz, while the official Piper
                    # MOVE_P demo is proven at 50 Hz. Drop redundant frames so
                    # MotionCtrl_2 + the two EndPose CAN frames cannot overrun
                    # the gs_usb transmit path.
                    send_now = (
                        last_pose_target is None or
                        time.monotonic() - last_pose_send_time >=
                            MOVE_P_SEND_PERIOD
                    )
                    if send_now:
                        # First official MOVE_P packet latches the measured
                        # pose; subsequent packets use the fixed training Home
                        # reference. No reconnect-relative drift or old Home.
                        sent_pose = (current_pose if official_training_pose
                                     and entering_pose else target_pose)
                        # Dedicated override applies only to the named
                        # firmware-IK profile, never legacy pose, joint/IK,
                        # Z/H training Home or mechanical-zero shutdown.
                        pose_speed = (
                            official_move_p_speed_percent
                            if official_training_pose and
                            official_move_p_speed_percent is not None
                            else speed_percent
                        )
                        send_move_p(piper, sent_pose, pose_speed)
                        if pose_recovery is not None:
                            # Legacy firmware MOVE_P has a different owner;
                            # never restore an older MOVE_J goal in this mode.
                            pose_recovery.cancel()
                        last_pose_target = list(sent_pose)
                        last_pose_send_time = time.monotonic()
                    if control_mode != "pose":
                        print("✅ Piper official MOVE_P / EndPoseCtrl (50 Hz): firmware six-joint IK, with no custom IK or joint locks.", flush=True)
                    control_mode = "pose"
                    last_target = None
                    last_joint_send_time = 0.0
                    last_command = time.monotonic()
                    continue

                if command_type == "link6_offset":
                    hold_on_disconnect = False
                    xyz_offset_m = validate_link6_offset(command)
                    joint5_target_rad = validate_optional_joint5_target(command)
                    joint6_target_rad = validate_optional_joint6_target(command)
                    allow_all_joints = command.get("allow_all_joints") is True
                    allow_j5 = command.get("allow_j5") is True
                    vr_joint1_y_only = (
                        command.get("vr_joint1_y_only") is True
                    )
                    joint1_y_only = (
                        command.get("joint1_y_only") is True and
                        allow_all_joints
                    )
                    training_y_joint1 = (
                        command.get("ik_profile") == TRAINING_Y_J1_PROFILE)
                    training_j15 = (
                        command.get("ik_profile") == TRAINING_J15_PROFILE)
                    training_locked_wrist = (
                        command.get("ik_profile") in
                        (TRAINING_J123_PROFILE, TRAINING_Y_J1_PROFILE))
                    if training_locked_wrist:
                        # Named opt-in only: legacy VR/full-joint clients
                        # retain their modes. No independent wrist or
                        # cylindrical Y path may override the training lock.
                        joint5_target_rad = joint6_target_rad = None
                        allow_all_joints = allow_j5 = False
                        vr_joint1_y_only = training_y_joint1
                        joint1_y_only = False
                    if training_j15:
                        # Named five-joint mode, not six-joint unlock. Ignore
                        # old cylindrical/wrist/manual flags: XYZ owns J1-J5
                        # and J6 remains fixed at training Home.
                        allow_all_joints, allow_j5 = True, False
                        joint5_target_rad = None
                        joint6_target_rad = TRAINING_HOME_JOINTS[5]
                        vr_joint1_y_only = joint1_y_only = False
                    if (control_mode != "link6_ik" or
                            last_link6_profile != command.get("ik_profile")):
                        processed_link6_xyz_request = None
                        last_link6_joint5_request = None
                        pending_j5_only_target = None
                        training_y_hold_active = False
                        last_link6_profile = command.get("ik_profile")
                    manual_j5_mode = (
                        allow_all_joints and joint5_target_rad is not None
                    )
                    wrist_yaw_mode = (
                        allow_all_joints and joint6_target_rad is not None
                    )
                    if manual_j5_mode:
                        if (
                            last_link6_joint5_request is not None and
                            abs(joint5_target_rad -
                                last_link6_joint5_request) > 1.0e-6
                        ):
                            # The keyboard arrow is a separate joint-space
                            # command.  Latch it until J5 reaches the target;
                            # do not ask Cartesian IK to compensate its link6
                            # displacement with the other five joints.
                            pending_j5_only_target = joint5_target_rad
                            # The XYZ sample carried by the same TCP frame is
                            # the existing Cartesian command, not a new move.
                            processed_link6_xyz_request = list(xyz_offset_m)
                            print(
                                "↕️ Independent J5 target: "
                                f"{math.degrees(joint5_target_rad):.1f} deg; "
                                "J1/J2/J3/J4/J6 hold their previous targets without rerunning IK.",
                                flush=True,
                            )
                        last_link6_joint5_request = joint5_target_rad
                    locked_joint_values = None
                    vr_joint1_target_rad = None
                    joint1_target_rad = None
                    if training_j15:
                        active_joints = LINK6_IK_WRIST_YAW_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_WRIST_YAW_LOCKED_JOINTS
                        locked_joint_values = {5: TRAINING_HOME_JOINTS[5]}
                        joint_mode_text = (
                            "five-joint XYZ IK using J1-J5; J6 fixed at the "
                            "training-Home 0 deg; official joint limits retained"
                        )
                    elif training_y_joint1:
                        vr_joint1_target_rad = math.atan2(
                            xyz_offset_m[1], TRAINING_HOME_LINK6_XYZ_M[0])
                        lower_j1, upper_j1 = JOINT_LIMITS_RAD[0]
                        vr_joint1_target_rad = max(
                            lower_j1 + 1.0e-4,
                            min(upper_j1 - 1.0e-4, vr_joint1_target_rad))
                        active_joints = LINK6_IK_VR_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_VR_LOCKED_JOINTS
                        locked_joint_values = {
                            0: vr_joint1_target_rad,
                            3: TRAINING_HOME_JOINTS[3],
                            4: TRAINING_HOME_JOINTS[4],
                            5: TRAINING_HOME_JOINTS[5],
                        }
                        joint_mode_text = (
                            "decoupled control: Y uses J1 only, X/Z use J2/J3, "
                            "and J4-J6 are locked; pure-Y commands hold the other five joints"
                        )
                    elif training_locked_wrist:
                        active_joints = LINK6_IK_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_LOCKED_JOINTS
                        locked_joint_values = {
                            joint: TRAINING_HOME_JOINTS[joint]
                            for joint in locked_joints
                        }
                        joint_mode_text = (
                            "training-constrained IK: XYZ uses J1-J3 only; "
                            "J4/J5/J6 locked at training Home [0,-69.9,0] deg"
                        )
                    elif vr_joint1_y_only:
                        # Treat the VR Y offset as a rotation about Piper J1.
                        # The reference radius is fixed, so changing only Y
                        # cannot make J2/J3 compensate. X remains the planar
                        # radial command and Z remains the vertical command.
                        vr_joint1_target_rad = math.atan2(
                            xyz_offset_m[1],
                            TRAINING_HOME_LINK6_XYZ_M[0],
                        )
                        lower_j1, upper_j1 = JOINT_LIMITS_RAD[0]
                        vr_joint1_target_rad = max(
                            lower_j1 + 1.0e-4,
                            min(upper_j1 - 1.0e-4,
                                vr_joint1_target_rad),
                        )
                        active_joints = LINK6_IK_VR_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_VR_LOCKED_JOINTS
                        locked_joint_values = {
                            0: vr_joint1_target_rad,
                        }
                        joint_mode_text = (
                            "VR wrist-locked IK: X/Z use J2/J3 only, Y uses J1 only; "
                            "J4/J5/J6 locked at training Home"
                        )
                    elif joint1_y_only:
                        # Keyboard control is cylindrical rather than
                        # Cartesian in Y: the Y command maps only to base J1.
                        # Its reference radius is fixed, so X changes never
                        # feed back into the J1 target.
                        joint1_target_rad = math.atan2(
                            xyz_offset_m[1],
                            TRAINING_HOME_LINK6_XYZ_M[0],
                        )
                        lower_j1, upper_j1 = JOINT_LIMITS_RAD[0]
                        joint1_target_rad = max(
                            lower_j1 + 1.0e-4,
                            min(upper_j1 - 1.0e-4, joint1_target_rad),
                        )
                        active_joints = LINK6_IK_J1_Y_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_J1_Y_LOCKED_JOINTS
                        locked_joint_values = {
                            0: joint1_target_rad,
                            5: (
                                joint6_target_rad
                                if joint6_target_rad is not None
                                else TRAINING_HOME_JOINTS[5]
                            ),
                        }
                        joint_mode_text = (
                            "decoupled IK: Y uses J1 only, X/Z use J2-J5; independent J6 yaw="
                            f"{math.degrees(locked_joint_values[5]):.1f} deg"
                        )
                    elif manual_j5_mode:
                        # J5 is excluded from Cartesian IK.  I/K/J/L/U/O use
                        # the other five joints; arrow-key changes are handled
                        # below as pure J5 joint-space commands.
                        active_joints = LINK6_IK_MANUAL_J5_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_MANUAL_J5_LOCKED_JOINTS
                        locked_joint_values = {4: joint5_target_rad}
                        joint_mode_text = (
                            "position IK with J1/J2/J3/J4/J6; J5 is independently controlled"
                        )
                    elif wrist_yaw_mode:
                        # Layered yaw correction: keep the requested link6 XYZ
                        # while J6 directly owns local tool-axis wrist yaw.
                        # This avoids Euler-yaw IK near the 88.4 degree Home
                        # pitch singularity.
                        active_joints = LINK6_IK_WRIST_YAW_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_WRIST_YAW_LOCKED_JOINTS
                        locked_joint_values = {5: joint6_target_rad}
                        joint_mode_text = (
                            "position IK with J1-J5; independent local J6 yaw="
                            f"{math.degrees(joint6_target_rad):.1f} deg"
                        )
                    elif allow_all_joints:
                        active_joints = LINK6_IK_ALL_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_ALL_LOCKED_JOINTS
                        joint_mode_text = "full J1-J6 solve with official joint limits retained"
                    elif allow_j5:
                        # Backward compatibility with an older keyboard client.
                        active_joints = LINK6_IK_J5_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_J5_LOCKED_JOINTS
                        joint_mode_text = "J1/J2/J3/J5 solved, J4/J6 locked"
                    else:
                        active_joints = LINK6_IK_ACTIVE_JOINTS
                        locked_joints = LINK6_IK_LOCKED_JOINTS
                        joint_mode_text = "J1-J3 solved, J4-J6 locked"
                    if control_mode != "link6_ik":
                        current = read_joint_radians(piper)
                        training_error = max(
                            abs(a - b) for a, b in
                            zip(current, TRAINING_HOME_JOINTS)
                        )
                        if (training_error > math.radians(8.0) and
                                not allow_all_joints and
                                not training_locked_wrist):
                            raise RuntimeError(
                                "Piper is not at training Home; joint6 IK startup rejected: "
                                f"maximum joint error {math.degrees(training_error):.1f} deg"
                            )
                        last_target = current
                        last_link6_send_time = 0.0
                        if (allow_all_joints and
                                training_error > math.radians(8.0)):
                            print(
                                "♻️ Piper full-joint IK resumed from the current pose; "
                                "the training-Home startup restriction was removed, maximum joint deviation="
                                f"{math.degrees(training_error):.1f} deg.",
                                flush=True,
                            )
                        print(
                            f"✅ Piper link6 position IK: {joint_mode_text}; training Home="
                            "[0,145.5,-72.2,0,-69.9,0] deg, "
                            "link6 XYZ=(458.6,0.0,28.6) mm",
                            flush=True,
                        )

                    if vr_joint1_y_only or joint1_y_only:
                        radial_target_m = (
                            TRAINING_HOME_LINK6_XYZ_M[0] + xyz_offset_m[0]
                        )
                        base_yaw_target_rad = (
                            vr_joint1_target_rad
                            if vr_joint1_y_only else joint1_target_rad
                        )
                        target_xyz_m = [
                            radial_target_m * math.cos(
                                base_yaw_target_rad),
                            radial_target_m * math.sin(
                                base_yaw_target_rad),
                            TRAINING_HOME_LINK6_XYZ_M[2] + xyz_offset_m[2],
                        ]
                    else:
                        target_xyz_m = [
                            TRAINING_HOME_LINK6_XYZ_M[index] +
                            xyz_offset_m[index]
                            for index in range(3)
                        ]
                    send_now = (
                        last_link6_send_time == 0.0 or
                        time.monotonic() - last_link6_send_time >=
                            link6_send_period
                    )
                    if send_now:
                        seed = (
                            last_target if last_target is not None
                            else read_joint_radians(piper)
                        )
                        ik_seed = list(seed)
                        if training_y_joint1:
                            ik_seed[3:] = TRAINING_HOME_JOINTS[3:]
                        if vr_joint1_y_only or joint1_y_only:
                            # Put the reach-search start on the requested J1
                            # ray. The later common rate limiter still compares
                            # against the real previous target, so J1 cannot
                            # jump in one command frame.
                            ik_seed[0] = (
                                vr_joint1_target_rad
                                if vr_joint1_y_only else joint1_target_rad
                            )
                        manual_j5_only_frame = False
                        training_y_only_frame = False
                        xyz_request_changed = (
                            processed_link6_xyz_request is None or
                            max(abs(a - b) for a, b in zip(
                                xyz_offset_m,
                                processed_link6_xyz_request,
                            )) > 1.0e-6
                        )
                        if training_y_joint1:
                            previous_request = (
                                processed_link6_xyz_request
                                if processed_link6_xyz_request is not None
                                else [0.0, 0.0, 0.0])
                            xz_changed = any(
                                abs(xyz_offset_m[i] - previous_request[i]) > 1.0e-6
                                for i in (0, 2))
                            if xz_changed:
                                training_y_hold_active = False
                            elif abs(xyz_offset_m[1] - previous_request[1]) > 1.0e-6:
                                # Latch until the next X/Z or explicit H.
                                # Repeated keepalive frames must NOT resume
                                # Cartesian IK and undo this J1-only motion.
                                training_y_hold_active = True
                        y_only_request_changed = (
                            joint1_y_only and
                            processed_link6_xyz_request is not None and
                            abs(xyz_offset_m[0] -
                                processed_link6_xyz_request[0]) <= 1.0e-6 and
                            abs(xyz_offset_m[1] -
                                processed_link6_xyz_request[1]) > 1.0e-6 and
                            abs(xyz_offset_m[2] -
                                processed_link6_xyz_request[2]) <= 1.0e-6
                        )
                        exact_training_home_requested = (
                            (allow_all_joints or vr_joint1_y_only or
                             training_locked_wrist) and
                            (
                                command.get("exact_training_home") is True or
                                (not training_y_joint1 and
                                    joint5_target_rad is None and
                                    (
                                        joint6_target_rad is None or
                                        abs(joint6_target_rad -
                                            TRAINING_HOME_JOINTS[5]) <= 1.0e-4
                                    ) and
                                    math.sqrt(sum(
                                        value * value
                                        for value in xyz_offset_m
                                    )) <= 1.0e-4
                                )
                            )
                        )
                        if exact_training_home_requested:
                            training_y_hold_active = False
                            pending_j5_only_target = None
                            target = list(TRAINING_HOME_JOINTS)
                            reachable_fraction = 1.0
                            _executed_xyz_m = list(
                                TRAINING_HOME_LINK6_XYZ_M)
                            processed_link6_xyz_request = list(xyz_offset_m)
                            if not exact_training_home_active:
                                print(
                                    "🏠 Exact H training Home: converging all six joints to "
                                    "[0,145.5,-72.2,0,-69.9,0] deg.",
                                    flush=True,
                                )
                            exact_training_home_active = True
                        else:
                            exact_training_home_active = False
                            if training_y_joint1 and training_y_hold_active:
                                target = list(seed)
                                target[0] = vr_joint1_target_rad
                                reachable_fraction = 1.0
                                _executed_xyz_m = link6_xyz_m(link6_fk, target).tolist()
                                processed_link6_xyz_request = list(xyz_offset_m)
                                training_y_only_frame = True
                            elif y_only_request_changed:
                                # Hard guarantee for J/L: alter J1 only. Keep
                                # the previous J2-J5 target and the requested
                                # independent J6 yaw without running IK.
                                target = list(seed)
                                target[0] = joint1_target_rad
                                if joint6_target_rad is not None:
                                    target[5] = joint6_target_rad
                                reachable_fraction = 1.0
                                _executed_xyz_m = link6_xyz_m(
                                    link6_fk, target).tolist()
                                processed_link6_xyz_request = list(
                                    xyz_offset_m)
                            elif (manual_j5_mode and
                                    pending_j5_only_target is not None):
                                # Arrow-key J5 is deliberately not an IK
                                # request: preserve J1/J2/J3/J4/J6 bit for bit
                                # and alter only the fifth joint target.
                                target = list(seed)
                                target[4] = pending_j5_only_target
                                locked_joint_values = {
                                    4: pending_j5_only_target
                                }
                                reachable_fraction = 1.0
                                _executed_xyz_m = link6_xyz_m(
                                    link6_fk, target).tolist()
                                manual_j5_only_frame = True
                            elif manual_j5_mode and not xyz_request_changed:
                                # An unchanged combined TCP frame must not
                                # cause a second Cartesian solve that undoes
                                # the displacement created by independent J5.
                                target = list(seed)
                                reachable_fraction = ik_reachable_fraction
                                _executed_xyz_m = link6_xyz_m(
                                    link6_fk, target).tolist()
                            elif training_j15:
                                target, reachable_fraction, _executed_xyz_m = (
                                    solve_training_j15_ik(
                                        link6_fk, ik_seed, target_xyz_m)
                                )
                                processed_link6_xyz_request = list(xyz_offset_m)
                            elif training_locked_wrist and not training_y_joint1:
                                target, reachable_fraction, _executed_xyz_m = (
                                    solve_training_j123_ik(
                                        link6_fk, ik_seed, target_xyz_m)
                                )
                                processed_link6_xyz_request = list(xyz_offset_m)
                            else:
                                target, reachable_fraction, _executed_xyz_m = (
                                    solve_link6_position_ik_reachable(
                                        link6_fk, ik_seed, target_xyz_m,
                                        active_joints, locked_joints,
                                        locked_joint_values
                                    )
                                )
                                processed_link6_xyz_request = list(
                                    xyz_offset_m)
                        ik_reachable_fraction = reachable_fraction
                        ik_unreachable = reachable_fraction < 0.999
                        # Keep projecting to the furthest reachable point and
                        # reporting ik_unreachable to the policy, but do not
                        # spam the Orin terminal with repeated workspace-boundary
                        # warnings while the operator holds the same command.
                        # Only an explicit training-Home request (S10 Z/H)
                        # gets the faster approach.  An ordinary zero-offset
                        # Cartesian command keeps the normal IK limits.
                        fast_training_home = (
                            exact_training_home_requested and
                            command.get("exact_training_home") is True
                        )
                        frame_max_step_rad = (
                            training_home_max_step_rad
                            if fast_training_home and
                            training_home_max_step_rad is not None
                            else max_step_rad
                        )
                        frame_speed_percent = (
                            training_home_speed_percent
                            if fast_training_home and
                            training_home_speed_percent is not None
                            else speed_percent
                        )
                        largest_step = max(
                            abs(a - b) for a, b in zip(target, seed)
                        )
                        if largest_step > frame_max_step_rad:
                            # The Cartesian target can require a larger joint
                            # change near a poorly conditioned configuration.
                            # Keep the session alive and approach it over
                            # multiple MOVE_J command frames instead of treating
                            # a valid IK result as a bridge failure.
                            target = [
                                old + max(
                                    -frame_max_step_rad,
                                    min(frame_max_step_rad, new - old),
                                )
                                for new, old in zip(target, seed)
                            ]
                            # Locked joints also respect the per-frame rate
                            # limit. For keyboard IK this is the requested J5
                            # target; older modes retain their Home lock.
                            for joint in locked_joints:
                                if training_y_only_frame and joint != 0:
                                    # Strict J/L semantics even during a
                                    # rate-limited transition: only J1 moves.
                                    continue
                                locked_target = (
                                    locked_joint_values[joint]
                                    if locked_joint_values is not None and
                                    joint in locked_joint_values
                                    else TRAINING_HOME_JOINTS[joint]
                                )
                                target[joint] = seed[joint] + max(
                                    -frame_max_step_rad,
                                    min(frame_max_step_rad,
                                        locked_target - seed[joint]),
                                )
                            now = time.monotonic()
                            if now - last_link6_limit_log_time >= 1.0:
                                print(
                                    "⚠️ Training-constrained IK remains slew-limited: "
                                    f"requested joint change {largest_step:.4f} rad, "
                                    f"this-frame maximum {frame_max_step_rad:.4f} rad; continuing toward the target.",
                                    flush=True,
                                )
                                last_link6_limit_log_time = now
                        if (manual_j5_only_frame and
                                pending_j5_only_target is not None and
                                abs(target[4] -
                                    pending_j5_only_target) <= 1.0e-6):
                            pending_j5_only_target = None
                        if not enabled:
                            enable_arm(piper)
                            enabled = True
                        send_move_j(piper, target, frame_speed_percent)
                        if pose_recovery is not None:
                            pose_recovery.remember(target)
                        last_target = target
                        last_link6_send_time = time.monotonic()
                    if control_mode != "link6_ik":
                        print(
                            f"✅ Piper enabled and entered link6 IK ({joint_mode_text}, "
                            f"{link6_rate_hz:g} Hz).",
                            flush=True,
                        )
                    control_mode = "link6_ik"
                    pose_reference = None
                    last_pose_target = None
                    last_pose_send_time = 0.0
                    last_joint_send_time = 0.0
                    last_command = time.monotonic()
                    continue

                target = validate_target(command.get("target"))
                startup_home = command.get("startup_home") is True
                frame_max_step_rad = (
                    training_home_max_step_rad
                    if startup_home and
                    training_home_max_step_rad is not None
                    else max_step_rad
                )
                frame_speed_percent = (
                    training_home_speed_percent
                    if startup_home and
                    training_home_speed_percent is not None
                    else speed_percent
                )
                hold_on_disconnect = False
                now = time.monotonic()
                send_now = (
                    last_joint_send_time == 0.0 or
                    now - last_joint_send_time >= JOINT_SEND_PERIOD
                )
                if send_now:
                    reference = (
                        last_target if last_target is not None
                        else read_joint_radians(piper)
                    )
                    largest_step = max(
                        abs(a - b) for a, b in zip(target, reference)
                    )
                    # A transient policy/reconnect discontinuity must not tear
                    # down the TCP session.  Slew each sent joint by at most the
                    # existing safety limit and continue toward the requested
                    # target on later 50 Hz frames.
                    if largest_step > frame_max_step_rad:
                        target = [
                            old + max(
                                -frame_max_step_rad,
                                min(frame_max_step_rad, new - old),
                            )
                            for new, old in zip(target, reference)
                        ]
                        if now - last_joint_limit_log_time >= 1.0:
                            print(
                                "⚠️ Piper joint target remains slew-limited: "
                                f"requested jump {largest_step:.4f} rad, "
                                f"this-frame limit {frame_max_step_rad:.4f} rad.",
                                flush=True,
                            )
                            last_joint_limit_log_time = now
                    if not enabled:
                        enable_arm(piper)
                        enabled = True
                    # A prior Cartesian command leaves the controller in MOVE_P.
                    # JointCtrl frames alone do not switch it back to MOVE_J.
                    if (control_mode != "joint" or
                            last_joint_speed_percent != frame_speed_percent):
                        piper.MotionCtrl_2(
                            0x01, 0x01, frame_speed_percent, 0x00)
                        mode_text = (
                            "Z startup pose 2.5x"
                            if startup_home else "normal slew limit"
                        )
                        print(
                            f"✅ Piper enabled and entered MOVE_J ({mode_text}, "
                            f"{frame_speed_percent}%, 50 Hz).",
                            flush=True,
                        )
                    piper.JointCtrl(*(
                        round(value * RAD_TO_MILLI_DEG) for value in target
                    ))
                    if pose_recovery is not None:
                        pose_recovery.remember(target)
                    control_mode = "joint"
                    last_joint_speed_percent = frame_speed_percent
                    last_target = target
                    last_joint_send_time = now
                    pose_reference = None
                    last_pose_target = None
                    last_pose_send_time = 0.0
                    last_link6_send_time = 0.0
                # Refresh the watchdog for every valid controller target, even
                # when this 200 Hz input frame is intentionally coalesced.
                last_command = time.monotonic()

        now = time.monotonic()
        if now >= next_state:
            if startup_zero is not None and startup_zero.active:
                startup_zero.step()
                last_command = time.monotonic()
            was_feedback_fresh = feedback_fresh
            feedback_error = None
            try:
                q = read_joint_radians(piper)
                feedback_fresh = True
            except RuntimeError as exc:
                q = list(previous_q)
                feedback_fresh = False
                feedback_error = str(exc)

            if recovery is not None:
                measured = recovery.observe()
                if measured is None:
                    feedback_fresh = False
                    feedback_error = recovery.reason
                if not recovery.physical_enabled:
                    enabled = False
                if (pose_recovery.goal is not None and
                        (not feedback_fresh or not recovery.physical_enabled or recovery.need_seed)):
                    if not pose_recovery.pending and now-last_command > watchdog_seconds:
                        pose_recovery.cancel()  # Expired policy intent cannot authorize rearm.
                    else:
                        pose_recovery.interrupt()
                if pose_recovery.pending:
                    recovery_motion_pending = False
                    restored = pose_recovery.step()
                    enabled = recovery.physical_enabled
                    last_command = time.monotonic()
                    if restored is not None:
                        last_target = restored
                        last_joint_send_time = last_link6_send_time = 0.0
                # Z sends one measured hold and waits for enabled=true. Finish
                # that authorized recovery handshake without requiring a second
                # target (or replaying the first requested trajectory). Intent
                # expires with the watchdog; idle sockets cannot rearm motors.
                if (recovery_motion_pending and not recovery.inhibited and
                        time.monotonic() - last_command <= watchdog_seconds):
                    ready, measured_seed = recovery.prepare()
                    if ready:
                        enabled = True
                        recovery_motion_pending = False
                        if measured_seed is not None:
                            last_target = measured_seed
                            control_mode = None
                            pose_reference = None
                            last_pose_target = None
                            last_pose_send_time = last_joint_send_time = 0.0
                            last_link6_send_time = 0.0
                            processed_link6_xyz_request = None
                            last_link6_joint5_request = None
                            pending_j5_only_target = None
                            training_y_hold_active = False

            if feedback_fresh and was_feedback_fresh:
                dt = max(now - previous_time, 1.0e-4)
                raw_dq = [(a - b) / dt for a, b in zip(q, previous_q)]
                dq = [max(-20.0, min(20.0, value)) for value in raw_dq]
            else:
                dq = [0.0] * 6

            if feedback_fresh != was_feedback_fresh:
                if feedback_fresh:
                    # The 103 client intentionally sends no new arm targets
                    # while feedback_fresh=false. Give it a full watchdog
                    # interval after recovery to receive this state and resume;
                    # elapsed time in the feedback gap is NOT target silence.
                    last_command = time.monotonic()
                    print(
                        "✅ Piper six-joint feedback recovered; continuing with the latest IK target.",
                        flush=True,
                    )
                else:
                    print(
                        f"⚠️ Piper joint feedback is unavailable ({feedback_error}); "
                        "holding the last target without disabling, pausing new motion, "
                        "and keeping TCP and quadruped communication independent.",
                        flush=True,
                    )
            # Observational only: age of the same SDK joint cache checked by
            # read_joint_radians. No enable/mode/target/timing changes.
            feedback_can_age_ms = None
            raw_q = None
            raw_feedback_fresh = False
            try:
                joint_cache = piper.GetArmJointMsgs()
                raw_values = raw_joint_message_radians(joint_cache)
                if all(math.isfinite(value) for value in raw_values):
                    raw_q = raw_values
                raw_feedback_fresh = joint_message_is_fresh(joint_cache)
                age_s = time.time() - joint_cache.time_stamp
                if joint_cache.Hz > 0 and math.isfinite(age_s) and age_s >= 0:
                    feedback_can_age_ms = age_s * 1000.0
            except Exception:
                pass
            official_pose_feedback = None
            if control_mode == "pose" and last_pose_profile == OFFICIAL_MOVE_P_PROFILE:
                try:
                    official_pose_feedback = read_end_pose(piper)
                except RuntimeError:
                    pass  # Diagnostic only; never invent successful IK.
            writer.queue({
                "type": "state",
                "t": now,
                "q": q,
                "dq": dq,
                "enabled": enabled,
                "auto_recovery": recovery is not None,
                "recovering": recovery is not None and (
                    not recovery.ready or pose_recovery.pending or
                    (startup_zero is not None and startup_zero.active)),
                "policy_pose_recovering": pose_recovery is not None and pose_recovery.pending,
                "startup_zero_active": startup_zero is not None and startup_zero.active,
                "startup_zero_complete": startup_zero is not None and startup_zero.complete,
                "recovery_reason": recovery.reason if recovery is not None else None,
                "feedback_fresh": feedback_fresh,
                "communication_alive": True,
                "feedback_error": feedback_error,
                "raw_q": raw_q,
                "raw_feedback_fresh": raw_feedback_fresh,
                "feedback_can_age_ms": feedback_can_age_ms,
                "ik_unreachable": ik_unreachable,
                "ik_reachable_fraction": ik_reachable_fraction,
                "control_mode": control_mode,
                "pose_profile": last_pose_profile if control_mode == "pose" else None,
                "official_move_p_profile": OFFICIAL_MOVE_P_PROFILE,
                "training_ik_profile": TRAINING_J123_PROFILE,
                "training_ik_profiles": [
                    TRAINING_J123_PROFILE, TRAINING_Y_J1_PROFILE],
                "ik_profile": last_link6_profile if control_mode == "link6_ik" else None,
                "pose_target_mm_deg": last_pose_target if control_mode == "pose" else None,
                "pose_feedback_mm_deg": official_pose_feedback,
            }, state=True)
            if feedback_fresh:
                previous_q = q
            previous_time = now
            next_state = now + period

        # Inspect feedback BEFORE the command watchdog. The controller stops
        # sending arm targets when CAN feedback is unavailable; that expected
        # pause must not clear the latched target/mode or disable the arm.
        # Keep the watchdog for genuine command silence with valid feedback,
        # and leave explicit disable, Ctrl+C, and actual TCP loss unchanged.
        if (enabled and feedback_fresh and not hold_on_disconnect and
                not (startup_zero is not None and startup_zero.active) and
                not (pose_recovery is not None and pose_recovery.pending) and
                time.monotonic() - last_command > watchdog_seconds):
            if pose_recovery is not None:
                pose_recovery.cancel()
            disable_arm(piper, f"no target received for {watchdog_seconds:.2f} seconds")
            enabled = False
            last_target = None
            pose_reference = None
            last_pose_target = None
            last_pose_send_time = 0.0
            last_joint_send_time = 0.0
            last_link6_send_time = 0.0
            processed_link6_xyz_request = None
            last_link6_joint5_request = None
            pending_j5_only_target = None
            control_mode = None


def main() -> int:
    parser = argparse.ArgumentParser(description="S10 whole-body Piper TCP bridge")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9999)
    parser.add_argument("--can", default=os.getenv("PIPER_CAN_PORT"),
                        help="SocketCAN device; defaults to gs_usb auto-discovery")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument(
        "--link6-rate", type=float, default=1.0 / LINK6_IK_SEND_PERIOD,
        help="constrained link6 IK/MOVE_J command rate in Hz (default: 50)",
    )
    parser.add_argument("--speed", type=int, default=30)
    parser.add_argument("--official-move-p-speed", type=int, default=None,
                        help="named official MOVE_P only: speed percent; other modes unchanged")
    parser.add_argument("--watchdog", type=float, default=1.00)
    parser.add_argument(
        "--allow-ip",
        default="10.21.41.1,10.21.31.103",
        help="comma-separated S10 AOS addresses allowed to connect",
    )
    parser.add_argument("--max-step", type=float, default=0.08,
                        help="maximum target change per received frame, rad")
    parser.add_argument("--training-home-speed", type=int, default=None,
                        help="MOVE_J speed percent for explicit training Home only")
    parser.add_argument("--training-home-max-step", type=float, default=None,
                        help="joint target change per frame for explicit training Home only, rad")
    parser.add_argument("--home-speed", type=int, default=15,
                        help="startup zero-position MOVE_J speed percent")
    parser.add_argument("--home-timeout", type=float, default=30.0)
    parser.add_argument("--home-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--home-delay", type=float, default=3.0,
                        help="seconds available to cancel before startup homing")
    parser.add_argument("--no-home", action="store_true",
                        help="listen without moving Piper to joint zero")
    parser.add_argument(
        "--no-gripper-init", action="store_true",
        help="do not enable or move the gripper during server startup",
    )
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--auto-recover", action="store_true",
                        help="TCP stays available during startup zero and saved-policy-pose recovery; never clear faults")
    args = parser.parse_args()
    if (not 1 <= args.port <= 65535 or args.rate <= 0 or
            args.link6_rate <= 0 or not 1 <= args.speed <= 100):
        parser.error("invalid port, feedback rate, or speed")
    if not 1 <= args.home_speed <= 100:
        parser.error("home-speed must be in the range 1..100")
    if (args.official_move_p_speed is not None and
            not 1 <= args.official_move_p_speed <= 100):
        parser.error("official-move-p-speed must be in the range 1..100")
    if (args.training_home_speed is not None and
            not 1 <= args.training_home_speed <= 100):
        parser.error("training-home-speed must be in the range 1..100")
    if (args.training_home_max_step is not None and
            args.training_home_max_step <= 0):
        parser.error("training-home-max-step must be greater than 0")
    if (args.watchdog <= 0 or args.max_step <= 0 or args.home_timeout <= 0 or
            args.home_tolerance_deg <= 0 or args.home_delay < 0):
        parser.error("invalid watchdog, max-step, or home parameter")

    can_port = discover_can_port(args.can)
    ensure_can_up(can_port)
    piper = C_PiperInterface_V2(can_port, dh_is_offset=1)
    recovery = ArmRecovery(piper) if args.auto_recover and not args.check_only else None
    try:
        try:
            piper.ConnectPort()
        except (RuntimeError, OSError) as exc:
            if recovery is None:
                raise
            recovery._wait(f"initial CAN connection is not ready: {exc}")
        if recovery is not None:
            recovery.last_reopen = time.monotonic()  # Initial reader gets a 2s grace.
        q = (wait_for_stable_feedback(piper) if recovery is None
             else recovery.observe())
    except KeyboardInterrupt:
        # No enable/home/gripper startup command has been issued in this
        # phase. Do not turn an invalid-feedback cancellation into motion or
        # claim the motors were disabled; only close our receive connection.
        print("\nCtrl+C: startup feedback wait canceled; no enable or homing command was sent.", flush=True)
        try:
            piper.DisconnectPort(thread_timeout=0.1)
        except (RuntimeError, OSError) as exc:
            print(f"⚠️ Piper receive-connection cleanup did not complete: {exc}", flush=True)
        return 130
    if args.check_only:
        piper.DisconnectPort(thread_timeout=0.1)
        return 0

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    keyboard_fd: Optional[int] = None
    old_terminal = None
    shutdown_home_allowed = False
    startup_zero = None
    try:
        # Reserve the port before moving.  If another bridge already owns it,
        # fail without performing the startup motion.
        server.bind((args.host, args.port))

        if recovery is not None:
            if not args.no_home:
                startup_zero = StartupZero(
                    piper, recovery, args.home_speed, args.home_timeout,
                    args.home_tolerance_deg, args.home_delay,
                    initialize_gripper=not args.no_gripper_init)
        elif not args.no_home:
            print(
                f"⚠️ Piper will automatically home all six joints at low speed in {args.home_delay:.1f} seconds; "
                "J1/J2 move first, followed by J3-J6. Ensure the area and path are clear and the emergency stop is available. "
                "Press Ctrl+C to cancel.",
                flush=True,
            )
            time.sleep(args.home_delay)
            move_to_joint_zero(
                piper,
                speed_percent=args.home_speed,
                timeout=args.home_timeout,
                tolerance_deg=args.home_tolerance_deg,
                recovery=recovery,
            )

        # Every normal server start establishes the same deterministic
        # gripper state.  This changes only the gripper; the six arm joints
        # retain the startup behavior above.
        if not args.no_gripper_init and recovery is None:
            close_gripper(piper)

        server.listen(1)
        if recovery is not None:
            server.settimeout(0.1)
        shutdown_home_allowed = True
        if sys.stdin.isatty():
            keyboard_fd = sys.stdin.fileno()
            old_terminal = termios.tcgetattr(keyboard_fd)
            tty.setcbreak(keyboard_fd)
        allowed_ips = {
            value.strip() for value in args.allow_ip.split(",") if value.strip()
        }
        arm_enabled = False
        while True:
            try:
                conn, address = server.accept()
            except socket.timeout:
                if recovery is not None:
                    if startup_zero is not None and startup_zero.active:
                        startup_zero.step()
                    else:
                        recovery.observe()
                continue
            if allowed_ips and address[0] not in allowed_ips:
                print(f"Rejected non-S10 controller: {address[0]}", flush=True)
                conn.close()
                continue
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"🔗 Controller connected: {address[0]}:{address[1]}", flush=True)
            try:
                arm_enabled = serve_client(
                    conn, piper, args.rate, args.speed,
                    args.watchdog, args.max_step,
                    args.training_home_speed, args.training_home_max_step,
                    keyboard_fd,
                    initially_enabled=arm_enabled,
                    official_move_p_speed_percent=args.official_move_p_speed,
                    link6_rate_hz=args.link6_rate,
                    recovery=recovery,
                    startup_zero=startup_zero,
                )
            except (ConnectionError, BrokenPipeError, OSError, ValueError, TypeError,
                    RuntimeError, json.JSONDecodeError) as exc:
                if startup_zero is not None:
                    startup_zero.cancel()
                if recovery is not None:
                    recovery.suspend()
                disable_arm(piper, f"bridge error: {exc}")
                arm_enabled = False
            finally:
                conn.close()
    except ManualZeroRequested:
        print("✅ Manual B homing completed; closing the connection and disabling Piper.", flush=True)
    except KeyboardInterrupt:
        if not shutdown_home_allowed:
            print("\nCtrl+C: startup canceled.", flush=True)
        else:
            print(
                "\nCtrl+C in the Orin terminal: exiting and disabling without homing. "
                "Press B for manual homing; Ctrl+C from the policy controller still performs homing.",
                flush=True,
            )
    finally:
        if old_terminal is not None and keyboard_fd is not None:
            termios.tcsetattr(keyboard_fd, termios.TCSADRAIN, old_terminal)
        disable_arm(piper, "service exit")
        server.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        raise SystemExit(f"Piper bridge failed to start: {exc}") from None
