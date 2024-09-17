"""A demonstration Node for ME495."""


import rclpy
from rclpy.node import Node


class MyNode(Node):
    """Node that demonstrates ROS 2."""

    def __init__(self):
        """Create MyNode."""
        super().__init__('mynode')
        self.get_logger().info('My Node')
        self._tmr = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """Increment the count at a fixed frequency."""
        self.get_logger().debug('Timer Hit')


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
