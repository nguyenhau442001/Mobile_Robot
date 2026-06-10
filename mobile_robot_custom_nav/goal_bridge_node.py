#!/usr/bin/env python3
"""GoalBridgeNode — RViz → GoToGoal action bridge.

Subscribes to /goal_pose (geometry_msgs/PoseStamped), which RViz publishes
when the user drops the "2D Goal Pose" arrow, and forwards it as a GoToGoal
action goal.

If a goal is already in flight when a new /goal_pose arrives the previous
goal is cancelled before the new one is sent.

Usage
-----
  ros2 run mobile_robot_custom_nav goal_bridge_node

Parameters
----------
  action_name  (str,  default: 'go_to_goal')
  goal_topic   (str,  default: '/goal_pose')
"""
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mobile_robot_interfaces.action import GoToGoal
from math_utils import yaw_from_quaternion


class GoalBridgeNode(Node):
    """Bridges /goal_pose topic to the GoToGoal action server."""

    def __init__(self) -> None:
        """Initialise the action client, subscription, and parameters."""
        super().__init__('goal_bridge_node')

        self.declare_parameter('action_name', 'go_to_goal')
        self.declare_parameter('goal_topic', '/goal_pose')

        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value

        self._cb_group = ReentrantCallbackGroup()
        self._active_goal_handle: ClientGoalHandle | None = None

        self._action_client = ActionClient(
            self, GoToGoal, action_name,
            callback_group=self._cb_group,
        )

        self._sub = self.create_subscription(
            PoseStamped,
            goal_topic,
            self._on_goal_pose,
            10,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"GoalBridgeNode ready — listening on '{goal_topic}', "
            f"sending to action '{action_name}'."
        )

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        yaw = yaw_from_quaternion(pose.pose.orientation)
        self.get_logger().info(
            f'Received /goal_pose: ({pose.pose.position.x:.3f}, '
            f'{pose.pose.position.y:.3f}, yaw={math.degrees(yaw):.3f}°) '
            f'frame="{pose.header.frame_id}"'
        )

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Action server not available — goal dropped.')
            return

        self._cancel_active_goal()

        goal_msg = GoToGoal.Goal()
        goal_msg.pose = pose

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return

        self._active_goal_handle = goal_handle
        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        p = fb.current_pose.pose.position
        self.get_logger().debug(
            f'  dist_remaining={fb.distance_remaining:.3f} m  '
            f'pos=({p.x:.3f}, {p.y:.3f})'
        )

    def _on_result(self, future) -> None:
        self._active_goal_handle = None
        result = future.result().result
        if result.success:
            fp = result.final_pose.pose.position
            yaw = yaw_from_quaternion(result.final_pose.pose.orientation)
            self.get_logger().info(
                f'Goal reached. Final pose: ({fp.x:.3f}, {fp.y:.3f}, '
                f'yaw={math.degrees(yaw):.3f}°).'
            )
        else:
            self.get_logger().warn('Goal did not succeed.')

    def _cancel_active_goal(self) -> None:
        if self._active_goal_handle is None:
            return
        self.get_logger().info('Cancelling previous goal.')
        self._active_goal_handle.cancel_goal_async()
        self._active_goal_handle = None


def main(args=None) -> None:
    """Entry point: spin the GoalBridgeNode with a multi-threaded executor."""
    rclpy.init(args=args)
    node = GoalBridgeNode()
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
