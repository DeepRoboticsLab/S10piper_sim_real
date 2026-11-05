"""Minimal Piper forward kinematics used only by the local simulator.

This is an independent, dependency-free implementation of the standard
Denavit-Hartenberg transform.  It contains no vendor SDK code and never opens
CAN or creates a physical-arm interface.
"""

import math


def _multiply(left, right):
    return [
        sum(left[row * 4 + k] * right[k * 4 + column] for k in range(4))
        for row in range(4)
        for column in range(4)
    ]


def _link_transform(alpha, length, theta, offset):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return [
        ct, -st, 0.0, length,
        st * ca, ct * ca, -sa, -sa * offset,
        st * sa, ct * sa, ca, ca * offset,
        0.0, 0.0, 0.0, 1.0,
    ]


def _pose(matrix):
    x, y, z = matrix[3], matrix[7], matrix[11]
    sine_term = matrix[8]
    if sine_term < -0.9999:
        roll, pitch, yaw = math.atan2(matrix[1], matrix[5]), math.pi / 2, 0.0
    elif sine_term > 0.9999:
        roll, pitch, yaw = -math.atan2(matrix[1], matrix[5]), -math.pi / 2, 0.0
    else:
        pitch = math.atan2(-sine_term, math.hypot(matrix[0], matrix[4]))
        cosine_pitch = math.cos(pitch)
        yaw = math.atan2(matrix[4] / cosine_pitch, matrix[0] / cosine_pitch)
        roll = math.atan2(matrix[9] / cosine_pitch, matrix[10] / cosine_pitch)
    degrees = 180.0 / math.pi
    return [x, y, z, roll * degrees, pitch * degrees, yaw * degrees]


class C_PiperForwardKinematics:
    """Compatibility surface required by ``piper_arm_server.py`` in simulation."""

    def __init__(self, dh_is_offset=1):
        self._a = [0.0, 0.0, 285.03, -21.98, 0.0, 0.0]
        self._alpha = [0.0, -math.pi / 2, 0.0, math.pi / 2,
                       -math.pi / 2, math.pi / 2]
        if dh_is_offset:
            self._theta = [0.0, math.radians(-172.22), math.radians(-102.78),
                           0.0, 0.0, 0.0]
        else:
            self._theta = [0.0, math.radians(-174.22), math.radians(-100.78),
                           0.0, 0.0, 0.0]
        self._d = [123.0, 0.0, 0.0, 250.75, 0.0, 91.0]

    def CalFK(self, joints):
        if len(joints) != 6:
            raise ValueError("Piper FK requires six joint angles")
        cumulative = None
        poses = []
        for alpha, length, theta_offset, offset, joint in zip(
                self._alpha, self._a, self._theta, self._d, joints):
            transform = _link_transform(
                alpha, length, float(joint) + theta_offset, offset)
            cumulative = transform if cumulative is None else _multiply(
                cumulative, transform)
            poses.append(_pose(cumulative))
        return poses
