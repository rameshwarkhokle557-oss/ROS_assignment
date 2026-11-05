"""B3: drive forward and stop before an obstacle using the front 2D LiDAR."""
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class EmergencyStop(Node):

    def __init__(self):
        super().__init__('emergency_stop')
        scan_topic = self.declare_parameter('scan_topic', '/front_lidar/scan').value
        self.speed = self.declare_parameter('speed', 0.2).value
        # wanted gap between the FRONT OF THE CHASSIS and the obstacle (task: 0.3 - 0.6 m)
        self.stop_gap = self.declare_parameter('stop_gap', 0.45).value
        # the LiDAR sits at the chassis centre, so the chassis front is body_l / 2 ahead of it
        self.front_offset = self.declare_parameter('front_offset', 0.40).value
        # only rays within +/- window_deg of straight ahead count (side walls are ignored)
        self.window = math.radians(self.declare_parameter('window_deg', 20.0).value)
        # no scan for this long (wall-clock seconds) -> do not move
        self.scan_timeout = self.declare_parameter('scan_timeout', 1.0).value
        # more NaN rays than this fraction in the window -> scan is unreliable, do not move
        self.max_nan_fraction = self.declare_parameter('max_nan_fraction', 0.5).value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)

        self.last_scan_time = None
        self.front_min = math.inf   # closest valid range in the front window, seen from the LiDAR
        self.nan_fraction = 0.0
        self.stopped = False        # latched once an obstacle is too close

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, scan):
        front_min = math.inf
        nan_count = 0
        total = 0
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))   # wrap to [-pi, pi]
            if abs(angle) > self.window:
                continue
            total += 1
            if math.isnan(r):
                nan_count += 1       # invalid reading: ignore it, but count it
            elif r == -math.inf or r < scan.range_min:
                front_min = 0.0      # closer than the sensor can measure: assume touching
            elif r == math.inf or r > scan.range_max:
                continue             # no echo within range: nothing seen on this ray
            else:
                front_min = min(front_min, r)
        self.front_min = front_min
        self.nan_fraction = nan_count / total if total else 1.0
        self.last_scan_time = self.now()

    def on_timer(self):
        cmd = Twist()                # default: zero velocity
        if self.last_scan_time is None:
            self.get_logger().info('waiting for the first scan', throttle_duration_sec=2.0)
        elif self.now() - self.last_scan_time > self.scan_timeout:
            self.get_logger().warn('scan data is stale: holding still', throttle_duration_sec=2.0)
        elif self.nan_fraction > self.max_nan_fraction:
            self.get_logger().warn('front scan mostly invalid: holding still',
                                   throttle_duration_sec=2.0)
        elif not self.stopped:
            gap = self.front_min - self.front_offset
            if gap <= self.stop_gap:
                self.stopped = True
                self.get_logger().info(
                    f'STOP: LiDAR range {self.front_min:.2f} m, gap to chassis front {gap:.2f} m')
            else:
                cmd.linear.x = self.speed
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())   # best effort: send a zero command on exit
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
