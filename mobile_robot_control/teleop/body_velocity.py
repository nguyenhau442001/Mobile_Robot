"""BodyVelocity named tuple for (linear, angular) velocity commands."""
from typing import NamedTuple


class BodyVelocity(NamedTuple):
    """Linear and angular velocity command for a differential drive robot."""

    linear: float   # m/s,   forward (+) / backward (-)
    angular: float  # rad/s, counter-clockwise (+) / clockwise (-)
