"""B2: drive a square, open loop (timed commands) or closed loop (/odom feedback).

Run with: --ros-args -p use_sim_time:=true -p mode:=open|closed
Needs the simulation launched with ground_truth:=true (used to measure the real closing error).
"""
import csv
import math
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pose_of(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return (p.x, p.y, yaw_of(q))


class DriveSquare(Node):

    def __init__(self):
        super().__init__('drive_square')
        self.mode = self.declare_parameter('mode', 'open').value        # 'open' or 'closed'
        self.side = self.declare_parameter('side', 1.0).value           # metres
        self.speed = self.declare_parameter('speed', 0.2).value         # m/s
        self.turn_rate = self.declare_parameter('turn_rate', 0.5).value  # rad/s
        default_csv = os.path.expanduser(f'~/b2_{self.mode}.csv')
        csv_path = self.declare_parameter('csv', default_csv).value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 10)
        self.create_timer(0.02, self.on_timer)

        self.odom = None            # (x, y, yaw) from /odom
        self.truth = None           # (x, y, yaw) from /ground_truth/odom
        self.odom_start = None
        self.truth_start = None
        self.t0 = None
        self.phase = 0              # 0..7: forward, turn, forward, turn, ...
        self.phase_start_time = 0.0
        self.phase_start_pose = None
        self.target_yaw = 0.0
        self.done = False

        self.csv_file = open(csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['t', 'odom_x', 'odom_y', 'odom_yaw', 'truth_x', 'truth_y', 'truth_yaw'])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        self.odom = pose_of(msg)

    def on_truth(self, msg):
        self.truth = pose_of(msg)

    def start_phase(self, t):
        self.phase_start_time = t
        self.phase_start_pose = self.odom
        if self.phase % 2 == 1:     # a turn phase: aim 90 degrees further left
            self.target_yaw = wrap(self.target_yaw + math.pi / 2)
        kind = 'forward' if self.phase % 2 == 0 else 'turn'
        self.get_logger().info(f'phase {self.phase}: {kind}')

    def on_timer(self):
        if self.done or self.odom is None:
            return
        if self.truth is None:
            self.get_logger().warn('waiting for /ground_truth/odom (launch with ground_truth:=true)',
                                   throttle_duration_sec=2.0)
            return
        t = self.now()
        if self.t0 is None:
            self.t0 = t
            self.odom_start = self.odom
            self.truth_start = self.truth
            self.target_yaw = self.odom[2]
            self.start_phase(t)

        self.writer.writerow([f'{t - self.t0:.3f}'] + [f'{v:.4f}' for v in self.odom + self.truth])

        cmd = Twist()
        forward = self.phase % 2 == 0
        if self.mode == 'open':
            elapsed = t - self.phase_start_time
            if forward:
                cmd.linear.x = self.speed
                finished = elapsed >= self.side / self.speed
            else:
                cmd.angular.z = self.turn_rate
                finished = elapsed >= (math.pi / 2) / self.turn_rate
        else:
            x, y, yaw = self.odom
            err = wrap(self.target_yaw - yaw)
            if forward:
                sx, sy, _ = self.phase_start_pose
                remaining = self.side - math.hypot(x - sx, y - sy)
                cmd.linear.x = clamp(remaining, 0.05, self.speed)       # slow down near the corner
                cmd.angular.z = clamp(1.5 * err, -0.3, 0.3)             # keep the heading
                finished = remaining <= 0.01
            else:
                rate = clamp(2.0 * abs(err), 0.1, self.turn_rate)
                cmd.angular.z = rate if err > 0 else -rate
                finished = abs(err) <= 0.01
        self.pub.publish(cmd)

        if finished:
            self.phase += 1
            if self.phase == 8:
                self.finish()
            else:
                self.start_phase(t)

    def finish(self):
        self.pub.publish(Twist())
        self.done = True
        self.csv_file.close()

        def closing_error(start, end):
            return math.hypot(end[0] - start[0], end[1] - start[1]), math.degrees(wrap(end[2] - start[2]))

        o = closing_error(self.odom_start, self.odom)
        g = closing_error(self.truth_start, self.truth)
        self.get_logger().info(
            f'RESULT mode={self.mode}: /odom closing error {o[0] * 100:.1f} cm, {o[1]:.1f} deg | '
            f'ground-truth closing error {g[0] * 100:.1f} cm, {g[1]:.1f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
