#!/usr/bin/env python3
"""GoToGoal action client.

Sends a single GoToGoal action request to the server and streams feedback
until the goal completes (or is aborted/cancelled).

Usage
-----
  ros2 run mobile_robot_custom_nav go_to_goal_client <x> <y> [yaw] [frame_id]

Arguments
---------
  x         Target x position [m]
  y         Target y position [m]
  yaw       Target heading [rad]  (default: 0.0)
  frame_id  Coordinate frame of the goal (default: 'odom')

Examples
--------
  ros2 run mobile_robot_custom_nav go_to_goal_client 2.0 1.5
  ros2 run mobile_robot_custom_nav go_to_goal_client 2.0 1.5 1.57
  ros2 run mobile_robot_custom_nav go_to_goal_client 2.0 1.5 0.0 map
"""
import argparse
import math
import sys

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from mobile_robot_interfaces.action import GoToGoal
from math_utils import yaw_from_quaternion


class GoToGoalClient(Node):
    """One-shot action client: sends a goal, streams feedback, then exits."""

    ACTION_NAME = 'go_to_goal'

    def __init__(self, x: float, y: float, yaw: float = 0.0,
                 frame_id: str = 'odom') -> None:
        """Initialise the action client with the target pose."""
        super().__init__('go_to_goal_client')
        self._goal_x = x
        self._goal_y = y
        self._goal_yaw = yaw
        self._frame_id = frame_id
        self._done = False

        self._action_client = ActionClient(self, GoToGoal, self.ACTION_NAME)

    def send_goal(self) -> None:
        """Wait for the server, then send the goal asynchronously."""
        self.get_logger().info(
            f"Waiting for action server '{self.ACTION_NAME}'...")
        self._action_client.wait_for_server()

        goal_msg = GoToGoal.Goal()
        goal_msg.pose = self._build_pose_stamped()

        self.get_logger().info(
            f'Sending goal: ({self._goal_x:.3f}, {self._goal_y:.3f}, '
            f'yaw={self._goal_yaw:.3f} rad) in frame "{self._frame_id}".')

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(self._on_goal_accepted)

    @property
    def done(self) -> bool:
        """True once the goal has reached a terminal state."""
        return self._done

    def _on_goal_accepted(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server.')
            self._done = True
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        p = fb.current_pose.pose.position
        yaw = yaw_from_quaternion(fb.current_pose.pose.orientation)
        self.get_logger().info(
            f'Feedback: pos=({p.x:.3f}, {p.y:.3f}, yaw={math.degrees(yaw):.3f}°), '
            f'distance_remaining={fb.distance_remaining:.3f} m')

    def _on_result(self, future) -> None:
        result = future.result().result
        status = future.result().status

        if result.success:
            fp = result.final_pose.pose.position
            yaw = yaw_from_quaternion(result.final_pose.pose.orientation)
            self.get_logger().info(
                f'Goal succeeded. Final pose: ({fp.x:.3f}, {fp.y:.3f}, '
                f'yaw={math.degrees(yaw):.3f}°).')
        else:
            self.get_logger().error(
                f'Goal did not succeed (action status={status}).')

        self._done = True

    def _build_pose_stamped(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = float(self._goal_x)
        pose.pose.position.y = float(self._goal_y)
        pose.pose.orientation.z = math.sin(self._goal_yaw / 2.0)
        pose.pose.orientation.w = math.cos(self._goal_yaw / 2.0)
        return pose


def main(args=None) -> None:
    """Entry point: parse CLI args, send a goal, and spin until done."""
    parser = argparse.ArgumentParser(
        description='Send a GoToGoal action request.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('x',        type=float, help='Target x position [m]')
    parser.add_argument('y',        type=float, help='Target y position [m]')
    parser.add_argument('yaw',      type=float, nargs='?', default=0.0,
                        help='Target heading [rad] (default: 0.0)')
    parser.add_argument('frame_id', nargs='?', default='odom',
                        help='Goal coordinate frame (default: odom)')

    ros_args_start = (sys.argv.index('--ros-args')
                      if '--ros-args' in sys.argv else len(sys.argv))
    cli_args = sys.argv[1:ros_args_start]

    parsed = parser.parse_args(cli_args)

    rclpy.init(args=args)
    node = GoToGoalClient(parsed.x, parsed.y, parsed.yaw, parsed.frame_id)
    node.send_goal()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
