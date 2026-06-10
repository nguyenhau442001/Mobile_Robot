"""Integration test: math_utils functions under a live rclpy environment.

Verifies that yaw_from_quaternion, normalize_angle, and clamp produce correct
results when geometry_msgs types are resolved from a real ROS installation.
"""
import math
import unittest

from geometry_msgs.msg import Quaternion
import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.fixture
def generate_test_description():
    """Return an empty launch description with a ReadyToTest signal."""
    return (
        launch.LaunchDescription([
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestMathUtils(unittest.TestCase):
    """Active tests: run alongside the (empty) launch description."""

    @classmethod
    def setUpClass(cls):
        """Initialise rclpy once for the whole test class."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shut down rclpy after all tests in this class."""
        rclpy.shutdown()

    def setUp(self):
        """Create a fresh node for each test."""
        self.node = rclpy.create_node('test_math_utils')

    def tearDown(self):
        """Destroy the node after each test."""
        self.node.destroy_node()

    # ── yaw_from_quaternion ───────────────────────────────────────────────────

    def test_yaw_identity(self):
        """Identity quaternion gives yaw = 0."""
        from math_utils import yaw_from_quaternion
        q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.assertAlmostEqual(yaw_from_quaternion(q), 0.0, places=6)

    def test_yaw_90_degrees(self):
        """Quaternion for +90° yaw gives pi/2."""
        from math_utils import yaw_from_quaternion
        q = Quaternion(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4))
        self.assertAlmostEqual(yaw_from_quaternion(q), math.pi / 2, places=6)

    def test_yaw_minus_90_degrees(self):
        """Quaternion for -90° yaw gives -pi/2."""
        from math_utils import yaw_from_quaternion
        q = Quaternion(x=0.0, y=0.0, z=math.sin(-math.pi / 4), w=math.cos(math.pi / 4))
        self.assertAlmostEqual(yaw_from_quaternion(q), -math.pi / 2, places=6)

    def test_yaw_180_degrees(self):
        """Quaternion for 180° yaw gives ±pi."""
        from math_utils import yaw_from_quaternion
        q = Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
        self.assertAlmostEqual(abs(yaw_from_quaternion(q)), math.pi, places=6)

    # ── normalize_angle ───────────────────────────────────────────────────────

    def test_normalize_angle_in_range(self):
        """Angle already in [-pi, pi] is unchanged."""
        from math_utils import normalize_angle
        self.assertAlmostEqual(normalize_angle(1.0), 1.0, places=6)

    def test_normalize_angle_wraps_positive(self):
        """Angle > pi wraps into [-pi, pi]."""
        from math_utils import normalize_angle
        self.assertAlmostEqual(normalize_angle(3 * math.pi / 2), -math.pi / 2, places=6)

    def test_normalize_angle_wraps_negative(self):
        """Angle < -pi wraps into [-pi, pi]."""
        from math_utils import normalize_angle
        self.assertAlmostEqual(normalize_angle(-3 * math.pi / 2), math.pi / 2, places=6)

    def test_normalize_angle_zero(self):
        """Zero stays zero."""
        from math_utils import normalize_angle
        self.assertAlmostEqual(normalize_angle(0.0), 0.0, places=6)

    # ── clamp ─────────────────────────────────────────────────────────────────

    def test_clamp_within_limit(self):
        """Value within [-limit, limit] is returned unchanged."""
        from math_utils import clamp
        self.assertAlmostEqual(clamp(0.5, 1.0), 0.5, places=6)

    def test_clamp_positive_overflow(self):
        """Value above +limit is clamped to +limit."""
        from math_utils import clamp
        self.assertAlmostEqual(clamp(2.0, 1.0), 1.0, places=6)

    def test_clamp_negative_overflow(self):
        """Value below -limit is clamped to -limit."""
        from math_utils import clamp
        self.assertAlmostEqual(clamp(-2.0, 1.0), -1.0, places=6)

    def test_clamp_zero(self):
        """Zero clamped to any positive limit stays zero."""
        from math_utils import clamp
        self.assertAlmostEqual(clamp(0.0, 5.0), 0.0, places=6)
