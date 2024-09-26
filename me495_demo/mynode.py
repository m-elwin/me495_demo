"""A demonstration Node for ME495."""


from example_interfaces.msg import Int64
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class MyNode(Node):
    """
    Node that demonstrates ROS 2.

    Publishes
    ---------
    count : example_interfaces/msg/Int64 - the count that is incremented

    Parameters
    ----------
    increment : Integer - the amount to increment the count by each time

    Services
    --------
    reset (std_srv/srv/Empty) - reset the count

    """

    def __init__(self):
        """Create MyNode."""
        super().__init__('mynode')
        self.get_logger().info('My Node')
        self.declare_parameter('increment', 1)
        self._inc = self.get_parameter('increment').value
        self._pub = self.create_publisher(Int64, 'count', 10)
        self._tmr = self.create_timer(0.5, self.timer_callback)
        self._srv = self.create_service(Empty, 'reset', self.reset_callback)
        self._count = 0

    def timer_callback(self):
        """Increment the count at a fixed frequency."""
        self.get_logger().debug('Timer Hit')
        self._pub.publish(Int64(data=self._count))
        self._count += self._inc

    def reset_callback(self, request, response):
        """Reset the count to zero."""
        self.get_logger().info('RESET!')
        self._count = 0
        return response


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
