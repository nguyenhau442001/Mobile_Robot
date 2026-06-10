"""TeleopNode — bridges a teleop input source to the /cmd_vel topic."""
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import yaml

from .incremental_teleop import IncrementalTeleop

_DEFAULT_PARAMS = os.path.join(
    get_package_share_directory('mobile_robot_control'),
    'config', 'teleop_params.yaml',
)


def _load_yaml_defaults() -> dict:
    """Return the ros__parameters dict from the bundled YAML."""
    if not os.path.isfile(_DEFAULT_PARAMS):
        raise FileNotFoundError(
            f'teleop_params.yaml not found at {_DEFAULT_PARAMS}. '
            'Rebuild and source the workspace.'
        )
    with open(_DEFAULT_PARAMS) as f:
        data = yaml.safe_load(f)
    return data.get('teleop_node', {}).get('ros__parameters', {})


_PUBLISH_RATE_HZ = 20.0


class TeleopNode(Node):
    """ROS2 node that streams teleop commands to /cmd_vel."""

    def __init__(self) -> None:
        """Initialise the publisher, timer, and teleop input source."""
        super().__init__('teleop_node')

        _defaults = _load_yaml_defaults()
        self.declare_parameter('max_linear_velocity', _defaults['max_linear_velocity'])
        self.declare_parameter('max_angular_velocity', _defaults['max_angular_velocity'])
        self.declare_parameter('linear_step', _defaults['linear_step'])
        self.declare_parameter('angular_step', _defaults['angular_step'])

        self._input = IncrementalTeleop(
            max_linear_velocity=self.get_parameter('max_linear_velocity').value,
            max_angular_velocity=self.get_parameter('max_angular_velocity').value,
            linear_step=self.get_parameter('linear_step').value,
            angular_step=self.get_parameter('angular_step').value,
        )
        self._input.start()

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(1.0 / _PUBLISH_RATE_HZ, self._tick)

    def _tick(self) -> None:
        """Publish the latest (v, ω) from the input source as a Twist."""
        if not self._input.is_active():
            self._cmd_vel_pub.publish(Twist())
            rclpy.shutdown()
            return

        cmd = self._input.read_input()

        twist = Twist()
        twist.linear.x = cmd.linear
        twist.angular.z = cmd.angular
        self._cmd_vel_pub.publish(twist)


def main(args=None) -> None:
    """Entry point: spin the TeleopNode."""
    rclpy.init(args=args)
    node = TeleopNode()
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
