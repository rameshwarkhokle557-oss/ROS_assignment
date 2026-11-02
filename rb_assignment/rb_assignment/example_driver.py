"""Publish a slow forward /cmd_vel and log /odom. A starting point only."""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class ExampleDriver(Node):

    def __init__(self):
        super().__init__('example_driver')
        self.speed = self.declare_parameter('speed', 0.2).value
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.1, self.on_timer)
        self.last_log = 0.0

    def on_timer(self):
        msg = Twist()
        msg.linear.x = self.speed
        self.pub.publish(msg)

    def on_odom(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log < 1.0:
            return
        self.last_log = now
        p = msg.pose.pose.position
        self.get_logger().info(f'odom x={p.x:.2f} y={p.y:.2f}')


def main():
    rclpy.init()
    node = ExampleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
