"""Proportional (P) controller with acceleration limiting."""
from math_utils import clamp
from motion_controller.base_controller import RobotController


class ProportionalController(RobotController):
    """P controller with asymmetric acceleration / deceleration limits.

    Each call to compute_linear / compute_angular:
      1. Computes the unconstrained P output: clamp(gain * error, max_velocity)
      2. Rate-limits the step from the previous output using the appropriate
         acceleration or deceleration limit, scaled by dt.

    Asymmetric limits allow aggressive deceleration (short stopping distance)
    while keeping acceleration gentle (smooth start), matching physical robot
    constraints.

    Args:
        k_linear:                Proportional gain on distance error.
        k_angular:               Proportional gain on heading error.
        max_linear_velocity:     Forward/backward speed cap [m/s].
        max_angular_velocity:    Turn-rate cap [rad/s].
        max_linear_acceleration: Max rate of increase in linear velocity [m/s²].
        min_linear_deceleration: Max rate of decrease (negative) [m/s²].
        max_angular_acceleration: Max rate of increase in angular velocity [rad/s²].
        min_angular_deceleration: Max rate of decrease (negative) [rad/s²].
    """

    def __init__(
        self,
        k_linear: float,
        k_angular: float,
        max_linear_velocity: float,
        max_angular_velocity: float,
        max_linear_acceleration: float,
        min_linear_deceleration: float,
        max_angular_acceleration: float,
        min_angular_deceleration: float,
    ) -> None:
        """Initialise gains, velocity limits, and accel/decel bounds."""
        self._k_linear = k_linear
        self._k_angular = k_angular
        self._max_linear = max_linear_velocity
        self._max_angular = max_angular_velocity
        self._max_lin_accel = max_linear_acceleration
        self._min_lin_decel = min_linear_deceleration
        self._max_ang_accel = max_angular_acceleration
        self._min_ang_decel = min_angular_deceleration

        self._prev_linear = 0.0
        self._prev_angular = 0.0

    def compute_linear(self, distance: float, dt: float) -> float:
        """Return acceleration-limited forward velocity [m/s]."""
        target = clamp(self._k_linear * distance, self._max_linear)
        self._prev_linear = self._rate_limit(
            target, self._prev_linear, dt,
            self._max_lin_accel, self._min_lin_decel,
        )
        return self._prev_linear

    def compute_angular(self, heading_error: float, dt: float) -> float:
        """Return acceleration-limited angular velocity [rad/s]."""
        target = clamp(self._k_angular * heading_error, self._max_angular)
        self._prev_angular = self._rate_limit(
            target, self._prev_angular, dt,
            self._max_ang_accel, self._min_ang_decel,
        )
        return self._prev_angular

    def reset(self) -> None:
        """Reset velocity state to zero (call between goals)."""
        self._prev_linear = 0.0
        self._prev_angular = 0.0

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _rate_limit(
        target: float,
        previous: float,
        dt: float,
        max_accel: float,
        min_decel: float,
    ) -> float:
        """Clamp the step from previous to target using accel/decel limits.

        max_accel is positive (speeding up), min_decel is negative (slowing
        down). The step is bounded by [min_decel * dt, max_accel * dt].
        """
        delta = target - previous
        max_step = max_accel * dt  # positive bound (speeding up)
        min_step = min_decel * dt  # negative bound (slowing down, min_decel < 0)
        return previous + max(min_step, min(max_step, delta))
