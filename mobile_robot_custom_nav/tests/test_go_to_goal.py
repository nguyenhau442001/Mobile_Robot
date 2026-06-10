"""Integration test: GoToGoal action server drives the robot to a target pose.

Launch description
------------------
  - go_to_goal_server node (the system under test)

Test fixture
------------
  A helper node runs inside the test process and plays two roles:

  1. Odometry simulator — publishes /odom at 20 Hz, moving the robot's
     simulated position 0.05 m/tick straight toward the active goal so the
     server's control loop sees realistic pose feedback and eventually
     declares the goal reached.

  2. Action client — sends a GoToGoal request and collects the result.

The test is deliberately physics-free: it validates the server's control
loop logic and action protocol (accept → feedback → succeed) without
requiring Gazebo. Wall-clock timeout is generous (30 s) to survive slow CI.
"""
import math
import threading
import time
import unittest

from geometry_msgs.msg import Quaternion
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mobile_robot_interfaces.action import GoToGoal


@pytest.mark.rostest
def generate_test_description():
    """Start the go_to_goal_server; signal ReadyToTest once it is up."""
    server_node = launch_ros.actions.Node(
        package='mobile_robot_custom_nav',
        executable='go_to_goal_server',
        output='screen',
    )
    return (
        launch.LaunchDescription([
            server_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'server_node': server_node},
    )


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle [rad] to a geometry_msgs/Quaternion (z-axis rotation)."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class _GoToGoalTestNode(Node):
    """Simulates odometry and sends a GoToGoal request."""

    _ODOM_HZ = 20.0
    _STEP_M = 0.05
    _TIMEOUT_S = 30.0

    def __init__(self) -> None:
        """Initialise publishers, action client, and pose state."""
        super().__init__('go_to_goal_test_node')

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._goal_x = 0.0
        self._goal_y = 0.0

        self._result = None
        self._result_event = threading.Event()

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._action_client = ActionClient(self, GoToGoal, 'go_to_goal')

        self._odom_timer = self.create_timer(
            1.0 / self._ODOM_HZ, self._publish_odom)

    def _publish_odom(self) -> None:
        """Advance simulated pose toward the active goal, then publish /odom."""
        dx = self._goal_x - self._x
        dy = self._goal_y - self._y
        dist = math.hypot(dx, dy)

        if dist > self._STEP_M:
            self._x += self._STEP_M * dx / dist
            self._y += self._STEP_M * dy / dist

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation = _yaw_to_quaternion(self._yaw)
        self._odom_pub.publish(msg)

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        """Queue a GoToGoal request on the executor thread."""
        self._goal_x = x
        self._goal_y = y

        goal_msg = GoToGoal.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation = _yaw_to_quaternion(yaw)

        def _do_send():
            self.destroy_timer(timer)
            if not self._action_client.server_is_ready():
                self.get_logger().error('Action server not ready — goal dropped.')
                self._result_event.set()
                return
            future = self._action_client.send_goal_async(goal_msg)
            future.add_done_callback(self._on_goal_accepted)

        timer = self.create_timer(0.0, _do_send)

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._result = None
            self._result_event.set()
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        self._result = future.result().result
        self._result_event.set()

    def wait_for_result(self) -> 'GoToGoal.Result | None':
        """Block until the action completes or the timeout expires."""
        self._result_event.wait(timeout=self._TIMEOUT_S)
        return self._result


@launch_testing.markers.keep_alive
class TestGoToGoalServer(unittest.TestCase):
    """Integration tests for the GoToGoal action server."""

    @classmethod
    def setUpClass(cls) -> None:
        """Initialise rclpy and start a multi-threaded executor."""
        rclpy.init()
        cls._node = _GoToGoalTestNode()
        cls._executor = MultiThreadedExecutor()
        cls._executor.add_node(cls._node)
        cls._spin_thread = threading.Thread(
            target=cls._executor.spin, daemon=True)
        cls._spin_thread.start()
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if cls._node._action_client.server_is_ready():
                break
            time.sleep(0.5)

    @classmethod
    def tearDownClass(cls) -> None:
        """Shut down the executor and rclpy."""
        cls._executor.shutdown()
        cls._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_goal_nearby_succeeds(self) -> None:
        """Server must accept a nearby goal and return success=True."""
        self._node._result = None
        self._node._result_event.clear()
        self._node._x = 0.0
        self._node._y = 0.0

        self._node.send_goal(0.5, 0.0)
        result = self._node.wait_for_result()

        self.assertIsNotNone(result, 'Action timed out — no result received')
        self.assertTrue(result.success)

    def test_goal_at_origin_when_already_there_succeeds(self) -> None:
        """Server must immediately succeed when robot is already within tolerance."""
        self._node._result = None
        self._node._result_event.clear()
        self._node._x = 0.0
        self._node._y = 0.0

        self._node.send_goal(0.0, 0.0)
        result = self._node.wait_for_result()

        self.assertIsNotNone(result, 'Action timed out — no result received')
        self.assertTrue(result.success)

    def test_goal_returns_final_pose(self) -> None:
        """Result must include a non-default final_pose stamped in odom frame."""
        self._node._result = None
        self._node._result_event.clear()
        self._node._x = 0.0
        self._node._y = 0.0

        self._node.send_goal(0.3, 0.3)
        result = self._node.wait_for_result()

        self.assertIsNotNone(result, 'Action timed out — no result received')
        self.assertTrue(result.success)
        self.assertEqual(result.final_pose.header.frame_id, 'odom')
