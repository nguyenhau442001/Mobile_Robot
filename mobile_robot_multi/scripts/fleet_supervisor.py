#!/usr/bin/env python3
"""Minimal multi-robot fleet supervisor — detect and resolve path conflicts.

Watches every robot's /<robot>/plan (the global path published by Nav2's
planner_server). On a 1 Hz timer, pairwise-checks the active paths: if two
paths come within `CONFLICT_RADIUS` metres of each other at any point, the
LATER-started goal is canceled via the NavigateToPose action's cancel
service. The earlier robot keeps going. This is the "first-come-first-served"
fairness policy — simplest thing that prevents head-to-head collisions.

Robot list is auto-discovered from the ROS graph: any topic shaped
`/<one-level>/plan` typed `nav_msgs/msg/Path` is treated as a Nav2 robot.
So the supervisor adapts to whatever's actually running — no env var or
yaml config to keep in sync with the launches.

Usage:
    # Bring up gazebo + nav2 first, then in a fresh terminal:
    ros2 run mobile_robot_multi fleet_supervisor.py
    CONFLICT_RADIUS=0.8 ros2 run mobile_robot_multi fleet_supervisor.py

Environment:
    CONFLICT_RADIUS   metres (default 0.5)
    DISCOVER_TIMEOUT  seconds to wait for /<robot>/plan topics to appear
                      (default 10)
"""

import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


# Match the QoS that rclpy.action.ActionServer uses for the status topic.
ACTION_STATUS_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class FleetSupervisor(Node):
    def __init__(self, conflict_radius=0.5, check_period=1.0,
                 discover_timeout=10.0):
        super().__init__('fleet_supervisor')
        self.conflict_radius = conflict_radius

        # Discover robots from the ROS graph instead of trusting an env var
        # or yaml. Any /<one-level>/plan topic typed nav_msgs/msg/Path is a
        # Nav2 robot. Wait for the set to stabilize before locking it in.
        robots = self._discover_robots_from_graph(discover_timeout)
        if not robots:
            raise RuntimeError(
                f'No /<robot>/plan topics found after {discover_timeout:.0f}s. '
                'Is Nav2 up?'
            )
        self.robots = robots

        # Per-robot state
        self.paths = {r: None for r in robots}
        self.goal_first_seen = {r: None for r in robots}  # rclpy.Time when active
        self.canceled = set()  # robots we already issued a cancel to

        self.get_logger().info(
            f'Watching {len(robots)} robots: {robots} '
            f'(conflict_radius={conflict_radius:.2f} m)'
        )

        for r in robots:
            self.create_subscription(
                Path, f'/{r}/plan',
                lambda msg, robot=r: self._on_plan(robot, msg),
                10,
            )
            self.create_subscription(
                GoalStatusArray,
                f'/{r}/navigate_to_pose/_action/status',
                lambda msg, robot=r: self._on_status(robot, msg),
                ACTION_STATUS_QOS,
            )

        self.cancel_clients = {
            r: self.create_client(
                CancelGoal, f'/{r}/navigate_to_pose/_action/cancel_goal')
            for r in robots
        }

        self.create_timer(check_period, self._check_conflicts)

    def _discover_robots_from_graph(self, timeout):
        """Find /<robot>/plan topics on the graph; return the namespaces.

        Polls every 0.5 s. Returns as soon as the set is non-empty AND
        unchanged across two consecutive scans (stability check — graph
        discovery is gossip-based and topics trickle in). Returns whatever
        was last seen if `timeout` expires first.
        """
        self.get_logger().info(
            f'Discovering robots via /<robot>/plan topics '
            f'(up to {timeout:.0f} s)...'
        )
        deadline = time.monotonic() + timeout
        last = set()
        while time.monotonic() < deadline:
            # Yield once so the rclpy graph cache refreshes.
            rclpy.spin_once(self, timeout_sec=0.0)
            current = self._scan_plan_topics()
            if current and current == last:
                return sorted(current)
            last = current
            time.sleep(0.5)
        return sorted(last)

    def _scan_plan_topics(self):
        found = set()
        for name, types in self.get_topic_names_and_types():
            if not name.endswith('/plan'):
                continue
            if 'nav_msgs/msg/Path' not in types:
                continue
            # We want exactly /<robot>/plan — not /plan and not deeper paths.
            parts = name.strip('/').split('/')
            if len(parts) == 2 and parts[1] == 'plan':
                found.add(parts[0])
        return found

    def _on_plan(self, robot, msg):
        self.paths[robot] = msg

    def _on_status(self, robot, msg):
        # Record the first time we observe an ACCEPTED/EXECUTING goal. When
        # the action becomes idle (no active goal), reset so a new dispatch
        # gets a fresh "first seen" timestamp.
        for s in msg.status_list:
            if s.status in (GoalStatus.STATUS_ACCEPTED,
                            GoalStatus.STATUS_EXECUTING):
                if self.goal_first_seen[robot] is None:
                    self.goal_first_seen[robot] = self.get_clock().now()
                    self.get_logger().info(f'{robot} goal active')
                return
        # No active goals → reset
        if self.goal_first_seen[robot] is not None:
            self.get_logger().info(f'{robot} goal idle')
        self.goal_first_seen[robot] = None
        self.paths[robot] = None  # stale plan, drop it
        self.canceled.discard(robot)

    def _check_conflicts(self):
        actives = [
            r for r in self.robots
            if self.paths[r] is not None
            and self.goal_first_seen[r] is not None
            and r not in self.canceled
        ]
        for i, r1 in enumerate(actives):
            for r2 in actives[i + 1:]:
                if self._paths_conflict(self.paths[r1], self.paths[r2]):
                    loser = (r1 if self.goal_first_seen[r1] >
                             self.goal_first_seen[r2] else r2)
                    winner = r2 if loser == r1 else r1
                    self.get_logger().warn(
                        f'CONFLICT: {r1} & {r2} paths cross within '
                        f'{self.conflict_radius:.2f} m — '
                        f'canceling {loser} (later goal), {winner} continues'
                    )
                    self._cancel(loser)

    def _paths_conflict(self, p1, p2):
        """Any pair of waypoints within conflict_radius => conflict.

        Subsample to ~50 points per path so this stays cheap on long plans.
        """
        r2 = self.conflict_radius ** 2
        step1 = max(1, len(p1.poses) // 50)
        step2 = max(1, len(p2.poses) // 50)
        for a in p1.poses[::step1]:
            ax, ay = a.pose.position.x, a.pose.position.y
            for b in p2.poses[::step2]:
                dx = ax - b.pose.position.x
                dy = ay - b.pose.position.y
                if dx * dx + dy * dy < r2:
                    return True
        return False

    def _cancel(self, robot):
        client = self.cancel_clients[robot]
        if not client.service_is_ready():
            self.get_logger().warn(
                f'Cancel service for {robot} not ready — skipping')
            return
        # Empty goal_info => cancel all active goals on this action server.
        client.call_async(CancelGoal.Request())
        self.canceled.add(robot)


def main():
    conflict_radius = float(os.environ.get('CONFLICT_RADIUS', 0.5))
    discover_timeout = float(os.environ.get('DISCOVER_TIMEOUT', 10.0))

    rclpy.init()
    try:
        node = FleetSupervisor(conflict_radius=conflict_radius,
                               discover_timeout=discover_timeout)
    except RuntimeError as e:
        print(f'[fleet_supervisor] {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    if len(node.robots) < 2:
        node.get_logger().warn(
            f'Only {len(node.robots)} robot found — supervisor needs >=2 '
            'to do anything useful. Exiting.'
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[fleet_supervisor] interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
