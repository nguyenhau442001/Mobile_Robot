#!/usr/bin/env python3
"""GoToGoal action server.

Drives the mobile robot to a target (x, y, yaw) pose. The server
subscribes to /odom for the robot's pose and publishes /cmd_vel to move it.

Velocity commands are delegated to any RobotController implementation via a
two-phase loop:

  Phase 1 — drive to (x, y): runs until distance <= goal_tolerance.
  Phase 2 — spin in place to goal_yaw: runs until |yaw_error| <= heading_tolerance.

Forward speed is suppressed while the heading error is large (turn first, then
drive). The loop publishes feedback (current_pose, distance_remaining) each tick.

The blocking loop relies on a MultiThreadedExecutor (see main) so the /odom
callback keeps updating the cached pose while the loop runs.

Gains, velocity limits, and tolerances are tunable ROS parameters; their
defaults live in config/go_to_goal_params.yaml (bundled into the package
share dir) and can be overridden at runtime via --ros-args -p.
"""
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
import yaml

from mobile_robot_interfaces.action import GoToGoal
from math_utils import normalize_angle
from math_utils import yaw_from_quaternion
from motion_controller import CascadedController


def _load_yaml(path: str, node_name: str) -> dict:
    """Return the ros__parameters dict for node_name from a YAML file."""
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f'Config file not found at {path}. '
            'Rebuild and source the workspace.'
        )
    with open(path) as f:
        data = yaml.safe_load(f)
    return data.get(node_name, {}).get('ros__parameters', {})


_SERVER_PARAMS = os.path.join(
    get_package_share_directory('mobile_robot_custom_nav'),
    'config', 'go_to_goal_params.yaml',
)
_CONTROLLER_PARAMS = os.path.join(
    get_package_share_directory('mobile_robot_control'),
    'config', 'cascaded_controller_params.yaml',
)


class GoToGoalServer(Node):
    """Action server that drives the robot to a target pose."""

    ACTION_NAME = 'go_to_goal'
    ODOM_TOPIC = '/odom'
    CMD_VEL_TOPIC = '/cmd_vel'

    def __init__(self):
        """Initialise the action server, subscriptions, publisher, and controller."""
        super().__init__('go_to_goal_server')

        _server_defaults = _load_yaml(_SERVER_PARAMS, 'go_to_goal_server')
        self.declare_parameter('control_frequency', _server_defaults['control_frequency'])
        self.declare_parameter('goal_tolerance', _server_defaults['goal_tolerance'])
        self.declare_parameter('heading_tolerance', _server_defaults['heading_tolerance'])

        self.control_frequency = self.get_parameter('control_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value

        _ctrl_defaults = _load_yaml(_CONTROLLER_PARAMS, 'cascaded_controller')
        self._controller = CascadedController(
            k_linear=_ctrl_defaults['k_linear'],
            k_angular=_ctrl_defaults['k_angular'],
            k_vel_linear=_ctrl_defaults['k_vel_linear'],
            k_vel_angular=_ctrl_defaults['k_vel_angular'],
            max_linear_velocity=_ctrl_defaults['max_linear_velocity'],
            max_angular_velocity=_ctrl_defaults['max_angular_velocity'],
            max_linear_acceleration=_ctrl_defaults['max_linear_acceleration'],
            min_linear_deceleration=_ctrl_defaults['min_linear_deceleration'],
            max_angular_acceleration=_ctrl_defaults['max_angular_acceleration'],
            min_angular_deceleration=_ctrl_defaults['min_angular_deceleration'],
        )

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.current_v = 0.0
        self.current_omega = 0.0
        self._prev_x = None
        self._prev_y = None
        self._prev_yaw = None
        self._prev_odom_time = None

        self._cb_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.ODOM_TOPIC, self._on_odom, qos,
            callback_group=self._cb_group)
        self.cmd_pub = self.create_publisher(Twist, self.CMD_VEL_TOPIC, qos)

        self.action_server = ActionServer(
            self,
            GoToGoal,
            self.ACTION_NAME,
            execute_callback=self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"go_to_goal_server ready. Action='{self.ACTION_NAME}', "
            f"odom='{self.ODOM_TOPIC}', cmd_vel='{self.CMD_VEL_TOPIC}'. "
            f'Drives to the goal (tolerance={self.goal_tolerance:.3f} m).')

    def _on_odom(self, msg: Odometry):
        """Cache the latest robot pose and differentiate velocity from pose."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._prev_odom_time is not None:
            dt = stamp - self._prev_odom_time
            if dt > 0.0:
                dx = x - self._prev_x
                dy = y - self._prev_y
                self.current_v = (
                    math.hypot(dx, dy) / dt
                    * math.copysign(1.0, dx * math.cos(yaw) + dy * math.sin(yaw))
                )
                self.current_omega = normalize_angle(yaw - self._prev_yaw) / dt

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self._prev_x = x
        self._prev_y = y
        self._prev_yaw = yaw
        self._prev_odom_time = stamp

    def _execute_callback(self, goal_handle):
        """Drive the robot to the goal with a cascaded control loop."""
        goal = goal_handle.request.pose
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        goal_yaw = yaw_from_quaternion(goal.pose.orientation)
        self.get_logger().info(
            f'Goal received: ({goal_x:.3f}, {goal_y:.3f}, '
            f'yaw={math.degrees(goal_yaw):.3f}°) in frame '
            f'"{goal.header.frame_id}". Driving to goal.')

        dt = 1.0 / self.control_frequency
        self._controller.reset()

        # Phase 1: drive to (x, y)
        while rclpy.ok():
            if self.current_x is None:
                self.get_logger().warn(
                    'Waiting for /odom before driving...',
                    throttle_duration_sec=2.0)
                time.sleep(dt)
                continue

            dx = goal_x - self.current_x
            dy = goal_y - self.current_y
            distance = math.hypot(dx, dy)

            if distance <= self.goal_tolerance:
                break

            heading_error = normalize_angle(
                math.atan2(dy, dx) - self.current_yaw)

            self._controller.update_feedback(self.current_v, self.current_omega)
            cmd = Twist()
            cmd.angular.z = self._controller.compute_angular(heading_error, dt)
            if abs(heading_error) <= self.heading_tolerance:
                cmd.linear.x = self._controller.compute_linear(distance, dt)
            self.cmd_pub.publish(cmd)

            goal_handle.publish_feedback(self._build_feedback(distance))
            time.sleep(dt)

        # Phase 2: spin in place to reach goal_yaw
        while rclpy.ok():
            yaw_error = normalize_angle(goal_yaw - self.current_yaw)
            if abs(yaw_error) <= self.heading_tolerance:
                break

            self._controller.update_feedback(self.current_v, self.current_omega)
            cmd = Twist()
            cmd.angular.z = self._controller.compute_angular(yaw_error, dt)
            self.cmd_pub.publish(cmd)

            goal_handle.publish_feedback(self._build_feedback(0.0))
            time.sleep(dt)

        self.cmd_pub.publish(Twist())

        final_distance = (math.hypot(goal_x - self.current_x,
                                     goal_y - self.current_y)
                          if self.current_x is not None else float('nan'))
        final_yaw_error = (math.degrees(
                               normalize_angle(goal_yaw - self.current_yaw))
                           if self.current_yaw is not None else float('nan'))
        self.get_logger().info(
            f'Goal reached: within {final_distance:.3f} m, '
            f'{final_yaw_error:.3f}° of target. Stopping.')

        goal_handle.succeed()

        result = GoToGoal.Result()
        result.success = True
        result.final_pose = self._current_pose_stamped(goal.header.frame_id)
        return result

    def _build_feedback(self, distance_remaining: float) -> 'GoToGoal.Feedback':
        """Assemble a GoToGoal feedback message from the current pose."""
        feedback = GoToGoal.Feedback()
        feedback.current_pose = self._current_pose_stamped()
        feedback.distance_remaining = float(distance_remaining)
        return feedback

    def _current_pose_stamped(self, frame_id: str = 'odom') -> PoseStamped:
        """Build a PoseStamped (x, y, yaw) from the cached odom pose."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        if self.current_x is not None:
            pose.pose.position.x = self.current_x
            pose.pose.position.y = self.current_y
            pose.pose.orientation.z = math.sin(self.current_yaw / 2.0)
            pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        return pose


def main(args=None):
    """Entry point: spin the GoToGoalServer with a multi-threaded executor."""
    rclpy.init(args=args)
    node = GoToGoalServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
