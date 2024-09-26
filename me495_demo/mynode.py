"""A demonstration Node for ME495."""

import sys

from example_interfaces.msg import Int64
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MyNode(Node):
    """
    Node that demonstrates ROS 2.

    Publishes
    ---------
    count : example_interfaces/msg/Int64 - the count that is incremented

    Parameters
    ----------
    increment : Integer - the amount to increment the count by each time

    """

    def __init__(self):
        """Create MyNode."""
        super().__init__('mynode')
        self.get_logger().info('My Node')
        self.declare_parameter('increment', 1)
        self._inc = self.get_parameter('increment').value
        self._pub = self.create_publisher(Int64, 'count', 10)
        self._tmr = self.create_timer(0.5, self.timer_callback)
        self._count = 0

    def timer_callback(self):
        """Increment the count at a fixed frequency."""
        self.get_logger().debug('Timer Hit')
        self._pub.publish(Int64(data=self._count))
        self._count += self._inc


def main(args=None):
    """Entrypoint for the mynode ROS node."""
    try:
        with rclpy.init(args=args):
            node = MyNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main(sys.argv)
