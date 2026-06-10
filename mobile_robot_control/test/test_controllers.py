"""Unit tests for ProportionalController and CascadedController.

Tests run without a live ROS2 environment; both controllers are pure Python
and depend only on math_utils, which is available on sys.path after a
colcon build.
"""
import math
import unittest


def _make_proportional(**overrides):
    """Return a ProportionalController with safe defaults."""
    from motion_controller.proportional_controller import ProportionalController
    defaults = {
        'k_linear': 1.0,
        'k_angular': 2.0,
        'max_linear_velocity': 5.0,
        'max_angular_velocity': 33.33,
        'max_linear_acceleration': 1.0,
        'min_linear_deceleration': -5.0,
        'max_angular_acceleration': 6.67,
        'min_angular_deceleration': -33.33,
    }
    defaults.update(overrides)
    return ProportionalController(**defaults)


def _make_cascaded(**overrides):
    """Return a CascadedController with safe defaults."""
    from motion_controller.cascaded_controller import CascadedController
    defaults = {
        'k_linear': 0.5,
        'k_angular': 2.0,
        'k_vel_linear': 0.2,
        'k_vel_angular': 0.5,
        'max_linear_velocity': 5.0,
        'max_angular_velocity': 33.33,
        'max_linear_acceleration': 1.0,
        'min_linear_deceleration': -5.0,
        'max_angular_acceleration': 6.67,
        'min_angular_deceleration': -33.33,
    }
    defaults.update(overrides)
    return CascadedController(**defaults)


# ── ProportionalController ────────────────────────────────────────────────────

class TestProportionalControllerLinear(unittest.TestCase):
    """Tests for ProportionalController linear (forward/backward) axis."""

    def test_zero_distance_gives_zero(self):
        """Zero distance error must produce zero velocity command."""
        ctrl = _make_proportional()
        self.assertAlmostEqual(ctrl.compute_linear(0.0, 0.1), 0.0)

    def test_output_capped_at_max_velocity(self):
        """Large distance must not produce a command above max_linear_velocity."""
        ctrl = _make_proportional(k_linear=10.0, max_linear_velocity=5.0)
        cmd = 0.0
        for _ in range(200):
            cmd = ctrl.compute_linear(100.0, 0.1)
        self.assertLessEqual(cmd, 5.0 + 1e-9)

    def test_proportional_output_before_rate_limit(self):
        """First tick from rest: output is bounded by max_linear_acceleration * dt."""
        ctrl = _make_proportional(k_linear=1.0, max_linear_acceleration=1.0)
        cmd = ctrl.compute_linear(10.0, 0.1)
        self.assertAlmostEqual(cmd, 0.1, places=9)

    def test_ramps_up_smoothly(self):
        """Velocity increases monotonically from rest, bounded by accel limit."""
        ctrl = _make_proportional(max_linear_acceleration=1.0)
        prev = 0.0
        for _ in range(50):
            cmd = ctrl.compute_linear(10.0, 0.1)
            self.assertGreaterEqual(cmd, prev - 1e-9)
            prev = cmd

    def test_decelerates_aggressively(self):
        """From max speed, one tick with zero distance must drop sharply."""
        ctrl = _make_proportional(
            max_linear_acceleration=1.0,
            min_linear_deceleration=-5.0,
        )
        for _ in range(100):
            ctrl.compute_linear(100.0, 0.1)
        cmd_after = ctrl.compute_linear(0.0, 0.1)
        self.assertGreaterEqual(cmd_after, 5.0 - 0.5 - 1e-9)

    def test_reset_clears_state(self):
        """After reset(), controller behaves as if freshly constructed."""
        ctrl = _make_proportional()
        for _ in range(50):
            ctrl.compute_linear(10.0, 0.1)
        ctrl.reset()
        cmd = ctrl.compute_linear(10.0, 0.1)
        self.assertAlmostEqual(cmd, 0.1, places=9)


class TestProportionalControllerAngular(unittest.TestCase):
    """Tests for ProportionalController angular (turn) axis."""

    def test_zero_heading_error_gives_zero(self):
        """Zero heading error must produce zero angular velocity command."""
        ctrl = _make_proportional()
        self.assertAlmostEqual(ctrl.compute_angular(0.0, 0.1), 0.0)

    def test_positive_heading_produces_positive_omega(self):
        """Positive heading error (CCW) must produce positive omega."""
        ctrl = _make_proportional()
        cmd = ctrl.compute_angular(math.pi / 4, 0.1)
        self.assertGreater(cmd, 0.0)

    def test_negative_heading_produces_negative_omega(self):
        """Negative heading error (CW) must produce negative omega."""
        ctrl = _make_proportional()
        cmd = ctrl.compute_angular(-math.pi / 4, 0.1)
        self.assertLess(cmd, 0.0)

    def test_output_capped_at_max_angular_velocity(self):
        """Large heading error must not exceed max_angular_velocity."""
        ctrl = _make_proportional(k_angular=100.0, max_angular_velocity=33.33)
        cmd = 0.0
        for _ in range(200):
            cmd = ctrl.compute_angular(math.pi, 0.1)
        self.assertLessEqual(cmd, 33.33 + 1e-9)

    def test_angular_reset(self):
        """After reset(), first tick is accel-limited from zero."""
        ctrl = _make_proportional()
        for _ in range(50):
            ctrl.compute_angular(math.pi, 0.1)
        ctrl.reset()
        cmd = ctrl.compute_angular(math.pi, 0.1)
        expected_first_step = 6.67 * 0.1
        self.assertAlmostEqual(cmd, expected_first_step, places=9)


# ── CascadedController ────────────────────────────────────────────────────────

class TestCascadedControllerLinear(unittest.TestCase):
    """Tests for CascadedController linear (forward/backward) axis."""

    def test_zero_distance_gives_zero(self):
        """Zero distance error must produce zero velocity command."""
        ctrl = _make_cascaded()
        self.assertAlmostEqual(ctrl.compute_linear(0.0, 0.1), 0.0)

    def test_output_capped_at_max_linear_velocity(self):
        """Large distance must not produce a command above max_linear_velocity."""
        ctrl = _make_cascaded(k_linear=10.0, max_linear_velocity=5.0)
        cmd = 0.0
        for _ in range(200):
            ctrl.update_feedback(cmd, 0.0)
            cmd = ctrl.compute_linear(100.0, 0.1)
        self.assertLessEqual(cmd, 5.0 + 1e-9)

    def test_accel_limited_first_tick(self):
        """Inner-loop output from rest must respect max_linear_acceleration."""
        ctrl = _make_cascaded(max_linear_acceleration=1.0)
        cmd = ctrl.compute_linear(10.0, 0.1)
        self.assertLessEqual(abs(cmd), 1.0 * 0.1 + 1e-9)

    def test_velocity_feedback_drives_inner_loop(self):
        """Reporting a non-zero measured velocity should reduce the command."""
        ctrl = _make_cascaded(k_linear=1.0, k_vel_linear=1.0,
                              max_linear_acceleration=100.0)
        cmd_no_fb = ctrl.compute_linear(2.0, 0.1)

        ctrl2 = _make_cascaded(k_linear=1.0, k_vel_linear=1.0,
                               max_linear_acceleration=100.0)
        ctrl2.update_feedback(1.0, 0.0)
        cmd_with_fb = ctrl2.compute_linear(2.0, 0.1)

        self.assertGreater(cmd_no_fb, cmd_with_fb)

    def test_ramps_monotonically(self):
        """Velocity increases monotonically from rest, bounded by accel limit."""
        ctrl = _make_cascaded()
        prev = 0.0
        for _ in range(50):
            ctrl.update_feedback(prev, 0.0)
            cmd = ctrl.compute_linear(10.0, 0.1)
            self.assertGreaterEqual(cmd, prev - 1e-9)
            prev = cmd

    def test_reset_clears_state(self):
        """After reset(), first tick is accel-limited from zero."""
        ctrl = _make_cascaded()
        for _ in range(50):
            ctrl.update_feedback(0.0, 0.0)
            ctrl.compute_linear(10.0, 0.1)
        ctrl.reset()
        cmd = ctrl.compute_linear(10.0, 0.1)
        self.assertLessEqual(abs(cmd), 1.0 * 0.1 + 1e-9)


class TestCascadedControllerAngular(unittest.TestCase):
    """Tests for CascadedController angular (turn) axis."""

    def test_zero_heading_error_gives_zero(self):
        """Zero heading error must produce zero angular velocity command."""
        ctrl = _make_cascaded()
        self.assertAlmostEqual(ctrl.compute_angular(0.0, 0.1), 0.0)

    def test_positive_heading_produces_positive_omega(self):
        """Positive heading error (CCW) must produce positive omega."""
        ctrl = _make_cascaded()
        cmd = ctrl.compute_angular(math.pi / 4, 0.1)
        self.assertGreater(cmd, 0.0)

    def test_negative_heading_produces_negative_omega(self):
        """Negative heading error (CW) must produce negative omega."""
        ctrl = _make_cascaded()
        cmd = ctrl.compute_angular(-math.pi / 4, 0.1)
        self.assertLess(cmd, 0.0)

    def test_output_capped_at_max_angular_velocity(self):
        """Large heading error must not exceed max_angular_velocity."""
        ctrl = _make_cascaded(k_angular=100.0, max_angular_velocity=33.33)
        cmd = 0.0
        for _ in range(200):
            ctrl.update_feedback(0.0, cmd)
            cmd = ctrl.compute_angular(math.pi, 0.1)
        self.assertLessEqual(cmd, 33.33 + 1e-9)

    def test_angular_reset(self):
        """After reset(), first tick is accel-limited from zero."""
        ctrl = _make_cascaded()
        for _ in range(50):
            ctrl.update_feedback(0.0, 0.0)
            ctrl.compute_angular(math.pi, 0.1)
        ctrl.reset()
        cmd = ctrl.compute_angular(math.pi, 0.1)
        self.assertLessEqual(abs(cmd), 6.67 * 0.1 + 1e-9)


# ── RobotController ABC ───────────────────────────────────────────────────────

class TestRobotControllerInterface(unittest.TestCase):
    """Tests that both controllers satisfy the RobotController ABC contract."""

    def test_proportional_implements_abstract_interface(self):
        """Proportional controller must be an instance of RobotController."""
        from motion_controller.base_controller import RobotController
        ctrl = _make_proportional()
        self.assertIsInstance(ctrl, RobotController)

    def test_cascaded_implements_abstract_interface(self):
        """Cascaded controller must be an instance of RobotController."""
        from motion_controller.base_controller import RobotController
        ctrl = _make_cascaded()
        self.assertIsInstance(ctrl, RobotController)

    def test_controllers_are_interchangeable(self):
        """Both controllers accept the same (distance, dt) / (heading, dt) calls."""
        dt = 0.05
        for ctrl in (_make_proportional(), _make_cascaded()):
            with self.subTest(ctrl=type(ctrl).__name__):
                v = ctrl.compute_linear(1.5, dt)
                w = ctrl.compute_angular(0.3, dt)
                self.assertIsInstance(v, float)
                self.assertIsInstance(w, float)


if __name__ == '__main__':
    unittest.main()
