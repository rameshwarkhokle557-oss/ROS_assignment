#!/usr/bin/env python3
"""Turn the public /cmd_vel Twist into the TwistStamped the active drive controller expects."""
import math

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node


def clamp(value, limit):
    return max(-limit, min(limit, value))


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.out_topic = self.declare_parameter('out_topic', '/drive_controller/cmd_vel').value
        self.max_linear = self.declare_parameter('max_linear', 1.0).value
        self.max_angular = self.declare_parameter('max_angular', 1.5).value
        # Set both for steered drives. 0 disables the curvature limit.
        self.wheelbase = self.declare_parameter('wheelbase', 0.0).value
        self.max_steer = self.declare_parameter('max_steer', 0.0).value
        self.pub = self.create_publisher(TwistStamped, self.out_topic, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg):
        out = TwistStamped()
        # Stamp from the node clock: with use_sim_time this is sim time.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = clamp(msg.linear.x, self.max_linear)
        out.twist.linear.y = clamp(msg.linear.y, self.max_linear)
        out.twist.angular.z = clamp(msg.angular.z, self.max_angular)
        if self.wheelbase > 0.0 and self.max_steer > 0.0:
            max_yaw_rate = abs(out.twist.linear.x) * math.tan(self.max_steer) / self.wheelbase
            out.twist.angular.z = clamp(out.twist.angular.z, max_yaw_rate)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
