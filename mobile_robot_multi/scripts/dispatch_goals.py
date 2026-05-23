#!/usr/bin/env python3
"""Send hardcoded Nav2 goals to several namespaced robots simultaneously.

Reads `mobile_robot_multi/config/goals.yaml` (or the path in the GOALS_FILE
env var) and fires a NavigateToPose action goal to /<robot>/navigate_to_pose
for each entry — all in parallel. Useful for eyeballing how each robot's
planner + controller + costmap behave when several stacks are working at the
same time in the shared world.

Usage:
    ros2 run mobile_robot_multi dispatch_goals.py
    GOALS_FILE=/tmp/my_goals.yaml ros2 run mobile_robot_multi dispatch_goals.py
"""

import math
import os
import sys
import threading
import time

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory


def yaw_to_quat(yaw):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


class RobotGoalSender(Node):
    """One per robot — owns its action client and tracks its own status."""

    def __init__(self, robot, goal):
        super().__init__(f'goal_sender_{robot}')
        self.robot = robot
        self.goal = goal
        self._action_name = f'/{robot}/navigate_to_pose'
        self._client = ActionClient(self, NavigateToPose, self._action_name)
        self._goal_handle = None
        self.done = False
        self.status = None

    def send(self):
        self.get_logger().info(f'Waiting for {self._action_name} ...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Action server {self._action_name} unavailable')
            self.status = 'no-server'
            self.done = True
            return

        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.goal['x'])
        msg.pose.pose.position.y = float(self.goal['y'])
        qx, qy, qz, qw = yaw_to_quat(float(self.goal.get('yaw', 0.0)))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending goal x={msg.pose.pose.position.x:.2f} '
            f'y={msg.pose.pose.position.y:.2f} '
            f'yaw={self.goal.get("yaw", 0.0):.2f}'
        )
        send_future = self._client.send_goal_async(msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED by server')
            self.status = 'rejected'
            self.done = True
            return
        self._goal_handle = handle
        self.get_logger().info('Goal accepted')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        code = future.result().status
        self.status = _STATUS_NAMES.get(code, f'status:{code}')
        self.get_logger().info(f'Final: {self.status}')
        self.done = True

    def cancel(self):
        if self._goal_handle is not None and not self.done:
            self._goal_handle.cancel_goal_async()


def load_goals():
    override = os.environ.get('GOALS_FILE')
    if override:
        path = override
    else:
        pkg = get_package_share_directory('mobile_robot_multi')
        path = os.path.join(pkg, 'config', 'goals.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    print(f'[dispatch_goals] loaded {len(data["goals"])} goals from {path}')
    return data['goals']


def main():
    goals = load_goals()

    rclpy.init()
    senders = [RobotGoalSender(g['robot'], g) for g in goals]

    executor = MultiThreadedExecutor()
    for s in senders:
        executor.add_node(s)

    # Spin in a background thread so each sender's wait_for_server +
    # async callbacks fire concurrently. Without this, sender N has to
    # wait for sender 1's wait_for_server timeout before it even starts.
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Fan out the wait+send across threads so all robots are dispatched
    # simultaneously, not serially.
    send_threads = [threading.Thread(target=s.send, daemon=True) for s in senders]
    for t in send_threads:
        t.start()
    for t in send_threads:
        t.join()

    try:
        while rclpy.ok() and not all(s.done for s in senders):
            time.sleep(0.2)
    except KeyboardInterrupt:
        print('\n[dispatch_goals] interrupt — canceling pending goals')
        for s in senders:
            s.cancel()
        # Give cancel callbacks a moment to fire.
        time.sleep(2.0)

    print('\n=== Final results ===')
    for s in senders:
        print(f'  {s.robot:>10}: {s.status}')

    executor.shutdown()
    for s in senders:
        s.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(s.status == 'succeeded' for s in senders) else 1)


if __name__ == '__main__':
    main()
