"""A demonstration Node for ME495."""


import rclpy
from rclpy.node import Node


class MyNode(Node):
    """Node that demonstrates ROS 2."""

    def __init__(self):
        """Create MyNode."""
        super().__init__('mynode')
        self.get_logger().info('My Node')


def main(args=None):
    """Entrypoint for the mynode ROS node."""
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
