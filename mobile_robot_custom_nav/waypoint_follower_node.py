#!/usr/bin/env python3
"""Waypoint follower: drives through a YAML waypoint list via GoToGoal."""
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from mobile_robot_interfaces.action import GoToGoal
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import yaml


def _load_waypoints(path: str) -> list[dict]:
    """Parse the waypoints list from a YAML file."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Waypoint file not found: {path}')
    with open(path) as f:
        data = yaml.safe_load(f)
    raw = data.get('waypoints')
    if not raw:
        raise ValueError(f'No "waypoints" key found in {path}')
    waypoints = []
    for entry in raw:
        pt = entry['point']
        waypoints.append({
            'x': float(pt['x']),
            'y': float(pt['y']),
            'th': float(pt['th']),  # degrees, same convention as Service_Robot
        })
    return waypoints


class WaypointFollowerNode(Node):
    """Drives through a waypoint list sequentially via the GoToGoal action."""

    ACTION_NAME = 'go_to_goal'
    DONE_TOPIC = '/delivery_done'

    def __init__(self):
        """Declare parameters, create publisher/action client, start timer."""
        super().__init__('waypoint_follower')

        default_wp_file = os.path.join(
            get_package_share_directory('mobile_robot_custom_nav'),
            'config', 'waypoints_table1.yaml',
        )
        self.declare_parameter('waypoint_file', default_wp_file)
        self.declare_parameter('dwell_time', 5.0)
        self.declare_parameter('frame_id', 'odom')

        self._waypoint_file = self.get_parameter('waypoint_file').value
        self._dwell_time = self.get_parameter('dwell_time').value
        self._frame_id = self.get_parameter('frame_id').value

        self._done_pub = self.create_publisher(String, self.DONE_TOPIC, 10)
        self._action_client = ActionClient(self, GoToGoal, self.ACTION_NAME)

        self.get_logger().info(
            f'waypoint_follower ready. File: {self._waypoint_file}, '
            f'dwell={self._dwell_time}s, frame={self._frame_id}')

        # Run the delivery loop on a one-shot timer so the executor is fully
        # started before the first action call goes out.
        self._timer = self.create_timer(0.5, self._start_delivery)

    def _start_delivery(self):
        self._timer.cancel()
        try:
            waypoints = _load_waypoints(self._waypoint_file)
        except (FileNotFoundError, ValueError, KeyError) as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return

        self.get_logger().info(
            f'Loaded {len(waypoints)} waypoint(s). '
            f'Waiting for action server "{self.ACTION_NAME}"...')
        self._action_client.wait_for_server()

        for idx, wp in enumerate(waypoints):
            yaw_rad = math.radians(wp['th'])
            self.get_logger().info(
                f'[{idx + 1}/{len(waypoints)}] Navigating to '
                f'({wp["x"]:.2f}, {wp["y"]:.2f}, {wp["th"]:.1f} deg)...')

            success = self._send_goal_and_wait(wp['x'], wp['y'], yaw_rad)
            if not success:
                self.get_logger().error(
                    f'Waypoint {idx + 1} failed or was cancelled. Aborting.')
                return

            self.get_logger().info(
                f'[{idx + 1}/{len(waypoints)}] Arrived. '
                f'Dwelling for {self._dwell_time}s...')
            time.sleep(self._dwell_time)

        msg = String()
        msg.data = f'done:{len(waypoints)} waypoints'
        self._done_pub.publish(msg)
        self.get_logger().info(
            f'Delivery complete — all {len(waypoints)} waypoints visited. '
            f'Published to {self.DONE_TOPIC}.')

    def _send_goal_and_wait(self, x: float, y: float, yaw: float) -> bool:
        """Send one GoToGoal; block until the goal reaches a terminal state."""
        goal_msg = GoToGoal.Goal()
        goal_msg.pose = self._build_pose_stamped(x, y, yaw)

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        p = fb.current_pose.pose.position
        self.get_logger().info(
            f'  pos=({p.x:.2f}, {p.y:.2f}), '
            f'dist_remaining={fb.distance_remaining:.2f} m',
            throttle_duration_sec=1.0,
        )

    def _build_pose_stamped(
            self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
