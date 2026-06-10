"""Cascaded position-velocity controller for mobile robot navigation.

Two-loop structure per axis (linear and angular):

  Outer loop — position control:
    v_setpoint = clamp(k_linear  * distance,      max_linear_velocity)
    ω_setpoint = clamp(k_angular * heading_error, max_angular_velocity)

  Inner loop — velocity control:
    v_error  = v_setpoint  - v_actual
    ω_error  = ω_setpoint  - ω_actual
    v_cmd    = rate_limit(prev_v_cmd  + k_vel_linear  * v_error)
    ω_cmd    = rate_limit(prev_ω_cmd  + k_vel_angular * ω_error)

Call update_feedback(v_actual, omega_actual) each tick before compute_*
so the inner loop has fresh velocity measurements from odometry.
"""
from math_utils import clamp
from motion_controller.base_controller import RobotController


class CascadedController(RobotController):
    """Cascaded P(position) → P(velocity) controller.

    The outer loop converts position/heading errors into velocity setpoints.
    The inner loop drives measured velocity toward those setpoints, with the
    output rate-limited by the asymmetric accel/decel bounds.

    Call update_feedback() once per tick (before compute_*) to supply the
    latest measured velocities from odometry.

    Args:
        k_linear:                 Outer-loop gain on distance error.
        k_angular:                Outer-loop gain on heading error.
        k_vel_linear:             Inner-loop gain on linear velocity error.
        k_vel_angular:            Inner-loop gain on angular velocity error.
        max_linear_velocity:      Forward/backward speed cap [m/s].
        max_angular_velocity:     Turn-rate cap [rad/s].
        max_linear_acceleration:  Max rate of increase in linear velocity [m/s²].
        min_linear_deceleration:  Max rate of decrease (negative) [m/s²].
        max_angular_acceleration: Max rate of increase in angular velocity [rad/s²].
        min_angular_deceleration: Max rate of decrease (negative) [rad/s²].
    """

    def __init__(
        self,
        k_linear: float,
        k_angular: float,
        k_vel_linear: float,
        k_vel_angular: float,
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
        self._k_vel_linear = k_vel_linear
        self._k_vel_angular = k_vel_angular
        self._max_linear = max_linear_velocity
        self._max_angular = max_angular_velocity
        self._max_lin_accel = max_linear_acceleration
        self._min_lin_decel = min_linear_deceleration
        self._max_ang_accel = max_angular_acceleration
        self._min_ang_decel = min_angular_deceleration

        self._v_actual = 0.0
        self._omega_actual = 0.0
        self._prev_v_cmd = 0.0
        self._prev_omega_cmd = 0.0

    # ── velocity feedback (not part of the base interface) ───────────────────

    def update_feedback(self, v_actual: float, omega_actual: float) -> None:
        """Store measured velocities from odometry for use this tick.

        Args:
            v_actual:     Measured forward velocity [m/s].
            omega_actual: Measured angular velocity [rad/s].
        """
        self._v_actual = v_actual
        self._omega_actual = omega_actual

    # ── RobotController interface ─────────────────────────────────────────────

    def compute_linear(self, distance: float, dt: float) -> float:
        """Outer: v_setpoint from distance; inner: track v_setpoint with rate limit."""
        v_setpoint = clamp(self._k_linear * distance, self._max_linear)
        v_error = v_setpoint - self._v_actual
        raw = self._prev_v_cmd + self._k_vel_linear * v_error
        self._prev_v_cmd = self._rate_limit(
            clamp(raw, self._max_linear),
            self._prev_v_cmd, dt,
            self._max_lin_accel, self._min_lin_decel,
        )
        return self._prev_v_cmd

    def compute_angular(self, heading_error: float, dt: float) -> float:
        """Outer: ω_setpoint from heading error; inner: track ω_setpoint with rate limit."""
        omega_setpoint = clamp(self._k_angular * heading_error, self._max_angular)
        omega_error = omega_setpoint - self._omega_actual
        raw = self._prev_omega_cmd + self._k_vel_angular * omega_error
        self._prev_omega_cmd = self._rate_limit(
            clamp(raw, self._max_angular),
            self._prev_omega_cmd, dt,
            self._max_ang_accel, self._min_ang_decel,
        )
        return self._prev_omega_cmd

    def reset(self) -> None:
        """Reset all velocity state to zero (call between goals)."""
        self._v_actual = 0.0
        self._omega_actual = 0.0
        self._prev_v_cmd = 0.0
        self._prev_omega_cmd = 0.0

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _rate_limit(
        target: float,
        previous: float,
        dt: float,
        max_accel: float,
        min_decel: float,
    ) -> float:
        """Clamp the step from previous to target by accel/decel bounds."""
        delta = target - previous
        max_step = max_accel * dt  # positive bound (speeding up)
        min_step = min_decel * dt  # negative bound (slowing down)
        return previous + max(min_step, min(max_step, delta))
