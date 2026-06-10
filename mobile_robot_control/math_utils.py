"""Shared math utilities for the mobile_robot_control package."""
import math

from geometry_msgs.msg import Quaternion


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract the yaw angle [rad] from a geometry_msgs/Quaternion.

    Uses the ZYX Euler convention: only the yaw (rotation about z) is returned.
    Result is in the range [-pi, pi].
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, limit: float) -> float:
    """Clamp value to the symmetric range [-limit, +limit]."""
    return max(-limit, min(limit, value))


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
