"""A demonstration Node for ME495."""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MyNode(Node):
    """Node that demonstrates ROS 2."""

    def __init__(self):
        """Create MyNode."""
        super().__init__('mynode')
        self.get_logger().info('My Node')


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
