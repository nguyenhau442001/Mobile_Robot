#!/usr/bin/env python3
"""
ROS 2 Node: Trapezoidal Velocity Controller
-------------------------------------------
This node generates a trapezoidal velocity profile and publishes
linear velocity commands to the /cmd_vel topic.

The profile consists of three phases:
    1. **Acceleration Phase** - Linearly increases velocity from 0 to v_target
    2. **Cruise Phase**       - Maintains a constant target velocity
    3. **Deceleration Phase** - Smoothly decreases velocity to 0 using a cosine function

Why discretize?
---------------
Instead of directly sending the velocity set point (v_target) to the robot,
we gradually ramp up and ramp down the velocity over time (dt). This prevents
jerky motion and helps maintain robot stability.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Default time step (s) for discretizing the velocity profile
DEFAULT_DT = 0.01


def trapezoid_profile(v_target, t_acc, t_cruise, t_dec, dt=DEFAULT_DT):
    """
    Generate a velocity profile based on a trapezoidal motion curve.

    Args:
        v_target (float): Target velocity in m/s
        t_acc (float): Duration of the acceleration phase in seconds
        t_cruise (float): Duration of the constant cruise phase in seconds
        t_dec (float): Duration of the deceleration phase in seconds
        dt (float): Time step for discretization (sampling rate)

    Returns:
        list of tuples: A list of (time, velocity) pairs representing
                        the entire velocity profile.
    """
    profile = []
    t = 0.0

    # Ensure t_acc and t_dec are never zero to avoid division by zero errors
    t_acc = max(t_acc, 0.001)
    t_dec = max(t_dec, 0.001)

    # -----------------------------
    # 1. Acceleration Phase
    # -----------------------------
    # Velocity increases linearly from 0 to v_target
    while t < t_acc:
        v = (t / t_acc) * v_target  # Linear interpolation
        profile.append((t, min(v, v_target)))  # Cap at v_target
        t += dt

    # -----------------------------
    # 2. Cruise Phase
    # -----------------------------
    # Maintain velocity at v_target
    while t < t_acc + t_cruise:
        profile.append((t, v_target))
        t += dt

    # -----------------------------
    # 3. Deceleration Phase
    # -----------------------------
    # Smoothly reduce velocity using a cosine shape:
    # v(t) = 0.5 * v_target * (1 + cos(pi * tau))
    # Where tau goes from 0 -> 1 across the deceleration period
    while t < t_acc + t_cruise + t_dec:
        tau = (t - (t_acc + t_cruise)) / t_dec  # Normalize time to [0, 1]
        v = 0.5 * v_target * (1 + math.cos(math.pi * tau))
        profile.append((t, max(v, 0.0)))  # Ensure velocity never goes negative
        t += dt

    return profile


class TrapezoidProfileController(Node):
    """
    ROS 2 node that publishes a precomputed trapezoidal velocity profile
    on /cmd_vel using a wall timer to maintain real-time pacing.
    """

    def __init__(self):
        super().__init__('trapezoid_profile_controller')

        # Declare parameters so values can be overridden from launch / CLI
        self.declare_parameter('v_target', 5.0)
        self.declare_parameter('t_acc', 5.0)
        self.declare_parameter('t_cruise', 0.2)
        self.declare_parameter('t_dec', 0.8)
        self.declare_parameter('dt', DEFAULT_DT)
        self.declare_parameter('max_vel_limit', 6.0)

        v_target = self.get_parameter('v_target').value
        t_acc = self.get_parameter('t_acc').value
        t_cruise = self.get_parameter('t_cruise').value
        t_dec = self.get_parameter('t_dec').value
        self.dt = self.get_parameter('dt').value
        self.max_vel_limit = self.get_parameter('max_vel_limit').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.profile = trapezoid_profile(v_target, t_acc, t_cruise, t_dec, self.dt)
        self.get_logger().info(
            f'Generated velocity profile with {len(self.profile)} points')

        self.index = 0
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info('Starting trapezoidal velocity profile...')

    def _tick(self):
        if self.index >= len(self.profile):
            self.pub.publish(Twist())
            self.get_logger().info('Profile complete. Robot stopped.')
            self.timer.cancel()
            return

        _, velocity = self.profile[self.index]
        self.index += 1

        # Clamp velocity to safety limits
        safe_velocity = min(max(velocity, 0.0), self.max_vel_limit)

        msg = Twist()
        msg.linear.x = safe_velocity
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrapezoidProfileController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the robot completely stops at the end
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
