#!/usr/bin/env python3
"""Digital twin sync node — real-time passive replica of the master robot state.

Subscribes to every state topic published by the master robot and
re-publishes them under the /twin namespace.  This node never sends
commands to the master; it is a read-only mirror that lets consumers
(RViz, analytics, what-if tools) observe the twin independently.

Subscriptions  (master)       Publications  (twin)
──────────────────────────────────────────────────
/odom                    →    /twin/odom
/joint_states            →    /twin/joint_states
/scan                    →    /twin/scan
/imu                     →    /twin/imu
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan


class DigitalTwinSyncNode(Node):
    def __init__(self):
        super().__init__('digital_twin_sync_node')

        self._pubs = {
            'odom': self.create_publisher(Odometry, '/twin/odom', 10),
            'joint_states': self.create_publisher(JointState, '/twin/joint_states', 10),
            'scan': self.create_publisher(LaserScan, '/twin/scan', 10),
            'imu': self.create_publisher(Imu, '/twin/imu', 10),
        }

        self.create_subscription(Odometry, '/odom',
                                 lambda msg: self._pubs['odom'].publish(msg), 10)
        self.create_subscription(JointState, '/joint_states',
                                 lambda msg: self._pubs['joint_states'].publish(msg), 10)
        self.create_subscription(LaserScan, '/scan',
                                 lambda msg: self._pubs['scan'].publish(msg), 10)
        self.create_subscription(Imu, '/imu',
                                 lambda msg: self._pubs['imu'].publish(msg), 10)

        self.get_logger().info('Digital twin sync node started — mirroring master → /twin')


def main():
    rclpy.init()
    node = DigitalTwinSyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
