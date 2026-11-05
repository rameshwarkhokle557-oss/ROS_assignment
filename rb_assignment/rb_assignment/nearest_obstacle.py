"""B1: report the distance and bearing of the closest valid obstacle in a 2D LaserScan."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class NearestObstacle(Node):

    def __init__(self):
        super().__init__('nearest_obstacle')
        scan_topic = self.declare_parameter('scan_topic', '/front_lidar/scan').value
        # data = [distance_m, bearing_rad]; bearing 0 = straight ahead of the LiDAR, left is positive
        self.pub = self.create_publisher(Float32MultiArray, '/nearest_obstacle', 10)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        best_range = None
        best_index = -1
        for i, r in enumerate(scan.ranges):
            # skip inf / NaN (no return) and values outside the sensor's valid range
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            if best_range is None or r < best_range:
                best_range = r
                best_index = i

        if best_range is None:
            self.get_logger().warn('no valid range in scan', throttle_duration_sec=2.0)
            return

        bearing = scan.angle_min + best_index * scan.angle_increment
        msg = Float32MultiArray()
        msg.data = [float(best_range), float(bearing)]
        self.pub.publish(msg)
        self.get_logger().info(
            f'nearest: {best_range:.2f} m at {math.degrees(bearing):.1f} deg',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = NearestObstacle()
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
