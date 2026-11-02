#!/usr/bin/env python3
"""Acceptance tests for rb_sim.

    scripts/smoke_test.py --robot default

Starts sim.launch.py headless with ground truth, drives the robot, compares against
/ground_truth/odom, prints a result table and exits non-zero if any check fails.
The robot named full_sensors also runs the sensor and obstacle checks.
"""
import argparse
import importlib
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
import tf2_ros
import yaml

TWIST_DRIVES = ('diff_2w', 'skid_4w', 'mecanum')
ALL_MODELS = ('box_0p5', 'box_1m', 'cylinder_0p5', 'wall_2m', 'wall_4m', 'barrier', 'traffic_cone')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def roll_pitch_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Pose:
    """Planar pose with a continuously unwrapped yaw."""

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.yaw = 0.0
        self.roll = self.pitch = 0.0
        self.speed = self.yaw_rate = 0.0
        self.count = 0
        self._last = None

    def update(self, msg):
        p = msg.pose.pose.position
        self.x, self.y, self.z = p.x, p.y, p.z
        raw = yaw_of(msg.pose.pose.orientation)
        self._last = raw if self._last is None else self._last
        self.yaw += wrap(raw - self._last)
        self._last = raw
        self.roll, self.pitch = roll_pitch_of(msg.pose.pose.orientation)
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        self.yaw_rate = abs(w.z)
        self.count += 1

    def snap(self):
        return (self.x, self.y, self.yaw)


def body_delta(a, b):
    """Displacement from snapshot a to b in a's body frame: (forward, left, dyaw)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return dx * c + dy * s, -dx * s + dy * c, b[2] - a[2]


class Results:

    def __init__(self):
        self.rows = []

    def add(self, name, expected, actual, ok):
        self.rows.append((name, expected, actual, bool(ok)))
        print(f'  [{"PASS" if ok else "FAIL"}] {name}: {actual} (expected {expected})', flush=True)

    def within(self, name, actual, target, tol_frac):
        self.add(name, f'{target:.3f} +-{tol_frac * 100:.0f}%', f'{actual:.3f}',
                 abs(actual - target) <= abs(target) * tol_frac)

    def table(self):
        w = max(len(r[0]) for r in self.rows)
        print('\n' + 'check'.ljust(w) + '  result  actual  (expected)')
        for name, exp, act, ok in self.rows:
            print(f'{name.ljust(w)}  {"PASS" if ok else "FAIL"}    {act}  ({exp})')
        failed = sum(1 for r in self.rows if not r[3])
        print(f'\n{len(self.rows) - failed}/{len(self.rows)} checks passed')
        return failed == 0


class Tester(Node):

    def __init__(self, results, cfg):
        super().__init__('rb_smoke_test')
        self.results = results
        self.cfg = cfg
        self.sim_time = 0.0
        self.gt, self.odom = Pose(), Pose()
        self.max_z = self.max_tilt = 0.0
        self.track_stability = False
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_gt, 10)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom.update(m), 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_gt(self, msg):
        self.gt.update(msg)
        if self.track_stability:
            self.max_z = max(self.max_z, abs(self.gt.z))
            self.max_tilt = max(self.max_tilt, abs(self.gt.roll), abs(self.gt.pitch))

    # ---- helpers ----
    def sync(self, wall_seconds=0.4):
        """Drain queued callbacks. Blocking subprocess calls leave /clock stale, which would shorten
        the next timed segment."""
        end = time.time() + wall_seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def spin_for_sim(self, seconds, cmd=None, wall_limit=None):
        """Spin for `seconds` of sim time, republishing `cmd` at 20 Hz of sim time."""
        t_end = self.sim_time + seconds
        wall_end = time.time() + (wall_limit or seconds * 10 + 30)
        next_pub = self.sim_time
        while self.sim_time < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cmd is not None and self.sim_time >= next_pub:
                self.pub.publish(cmd)
                next_pub = self.sim_time + 0.05
            if time.time() > wall_end:
                raise RuntimeError('sim time stopped advancing')

    def wait_for(self, predicate, wall_timeout, what):
        end = time.time() + wall_timeout
        while not predicate():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() > end:
                raise RuntimeError(f'timeout waiting for {what}')

    def twist(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        return t

    def wait_ready(self):
        self.wait_for(lambda: self.pub.get_subscription_count() > 0, 120, '/cmd_vel subscriber')
        self.wait_for(lambda: self.odom.count > 5 and self.gt.count > 5 and self.sim_time > 0, 120,
                      '/odom and /ground_truth/odom')
        # The relay must also be connected to the controller, or early commands are lost.
        control = get_package_share_directory('rb_control')
        with open(os.path.join(control, 'config', 'drives.yaml')) as f:
            input_topic = yaml.safe_load(f)[self.cfg['drive']]['input_topic']
        self.wait_for(lambda: self.count_subscribers(input_topic) > 0, 120, input_topic + ' subscriber')
        self.spin_for_sim(2.0, self.twist())

    def segment(self, name, cmd, duration, settle=1.5):
        """Drive one segment. Returns start/end snapshots and speeds measured after settling."""
        self.sync()  # the robot is at rest here; refresh a clock left stale by blocking calls
        g0, o0 = self.gt.snap(), self.odom.snap()
        self.spin_for_sim(duration, cmd)
        self.spin_for_sim(settle, self.twist())
        g1, o1 = self.gt.snap(), self.odom.snap()
        print(f'{name}: true {body_delta(g0, g1)}, odom {body_delta(o0, o1)}, '
              f'residual speed {self.gt.speed:.3f} m/s {self.gt.yaw_rate:.3f} rad/s', flush=True)
        return {'true': body_delta(g0, g1), 'odom': body_delta(o0, o1),
                'speed': self.gt.speed, 'yaw_rate': self.gt.yaw_rate}

    def check_stop(self, name, seg):
        self.results.add(f'{name}: stop', 'v<0.02, w<0.02',
                         f'v={seg["speed"]:.3f} w={seg["yaw_rate"]:.3f}',
                         seg['speed'] < 0.02 and seg['yaw_rate'] < 0.02)

    # ---- motion checks ----
    def motion_checks(self, drive):
        r = self.results
        self.track_stability = True

        seg = self.segment('straight', self.twist(vx=0.5), 4.0)
        fwd, left, _ = seg['true']
        dist = math.hypot(fwd, left)
        r.within('straight: true distance (m)', dist, 2.0, 0.15)
        odom_dist = math.hypot(*seg['odom'][:2])
        r.add('straight: odom vs true', 'within 5%', f'odom {odom_dist:.3f} vs true {dist:.3f}',
              abs(odom_dist - dist) <= 0.05 * dist)
        self.check_stop('straight', seg)

        seg = self.segment('reverse', self.twist(vx=-0.3), 3.0)
        fwd, left, _ = seg['true']
        r.within('reverse: true distance (m)', math.hypot(fwd, left), 0.9, 0.15)
        r.add('reverse: direction', 'backwards', f'{fwd:.3f} m along body x', fwd < 0)
        self.check_stop('reverse', seg)

        seg = self.segment('arc', self.twist(vx=0.5, wz=0.5), 4.0)
        r.within('arc: true yaw (rad)', seg['true'][2], 2.0, 0.15)
        self.check_stop('arc', seg)

        if drive in TWIST_DRIVES:
            seg = self.segment('spin', self.twist(wz=1.0), 3.0)
            r.within('spin: true yaw (rad)', seg['true'][2], 3.0, 0.15)
            drift = math.hypot(*seg['true'][:2])
            r.add('spin: drift', '<0.05 m', f'{drift:.3f} m', drift < 0.05)
            self.check_stop('spin', seg)

        if drive == 'mecanum':
            seg = self.segment('strafe', self.twist(vy=0.3), 4.0)
            fwd, left, dyaw = seg['true']
            r.within('strafe: body-y distance (m)', left, 1.2, 0.15)
            r.add('strafe: body-x', '<0.1 m', f'{abs(fwd):.3f} m', abs(fwd) < 0.1)
            r.add('strafe: yaw', '<0.05 rad', f'{abs(dyaw):.3f} rad', abs(dyaw) < 0.05)
            self.check_stop('strafe', seg)

        if drive == 'ackermann':
            seg = self.segment('yaw at v=0', self.twist(wz=1.0), 2.0)
            fwd, left, dyaw = seg['true']
            moved = math.hypot(fwd, left)
            r.add('yaw at v=0: no motion', '<0.05 m, <0.05 rad', f'{moved:.3f} m, {abs(dyaw):.3f} rad',
                  moved < 0.05 and abs(dyaw) < 0.05)
            self.check_stop('yaw at v=0', seg)
            seg = self.segment('tight arc', self.twist(vx=0.2, wz=1.5), 4.0)
            r.within('tight arc: true yaw, clamped (rad)', seg['true'][2], 0.97, 0.15)
            self.check_stop('tight arc', seg)

        self.track_stability = False
        r.add('stability: |z|', '<0.02 m', f'{self.max_z:.4f} m', self.max_z < 0.02)
        r.add('stability: roll/pitch', '<0.05 rad', f'{self.max_tilt:.4f} rad', self.max_tilt < 0.05)

    def example_driver_check(self):
        g0 = self.gt.snap()
        proc = subprocess.Popen(['ros2', 'run', 'rb_assignment', 'example_driver', '--ros-args',
                                 '-p', 'use_sim_time:=true'], start_new_session=True,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            self.spin_for_sim(5.0)
        finally:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=10)
        self.spin_for_sim(2.0, self.twist())
        fwd, left, _ = body_delta(g0, self.gt.snap())
        # Stop sending: wait for the command timeout so later checks start at rest.
        self.spin_for_sim(1.0, self.twist())
        self.results.add('example_driver moves robot', '>0.3 m', f'{math.hypot(fwd, left):.3f} m',
                         math.hypot(fwd, left) > 0.3)

    # ---- sensor checks ----
    def bridged_topics(self):
        """(ros topic, msg class, sensor) for everything the bridge config publishes."""
        bridge_dir = os.path.join(get_package_share_directory('rb_bringup'), 'config', 'bridge')
        out = []
        for s in self.cfg.get('sensors') or []:
            with open(os.path.join(bridge_dir, s['type'] + '.yaml')) as f:
                entries = yaml.safe_load(f.read().replace('{name}', s['name']))
            for e in entries:
                pkg, _, cls = e['ros_type_name'].split('/')
                out.append((e['ros_topic_name'], getattr(importlib.import_module(pkg + '.msg'), cls), s))
        return out

    def sensor_checks_rest(self):
        r = self.results
        topics = self.bridged_topics()
        latest = {}
        subs = [self.create_subscription(cls, topic, lambda m, t=topic: latest.setdefault(t, m),
                                         qos_profile_sensor_data) for topic, cls, _ in topics]
        try:
            self.wait_for(lambda: all(t in latest for t, _, _ in topics), 30, 'all sensor topics')
        except RuntimeError:
            pass
        # Drop the subscriptions: image and cloud traffic would otherwise slow every later spin.
        for sub in subs:
            self.destroy_subscription(sub)
        missing = [t for t, _, _ in topics if t not in latest]
        r.add('sensors: all topics publish', f'{len(topics)} topics within 30 s',
              'all' if not missing else f'missing {missing}', not missing)
        self.spin_for_sim(1.0)

        for topic, cls, s in topics:
            if topic not in latest:
                continue
            name, kind = s['name'], s['type']
            frame = latest[topic].header.frame_id
            if kind in ('rgb', 'rgbd') and not topic.endswith('/points'):
                expect = f'{name}_optical_frame'
            else:
                expect = f'{name}_link'
            r.add(f'frame: {topic}', expect, frame, frame == expect)

        for s in self.cfg['sensors']:
            if s['type'] == 'imu' and f'/{s["name"]}/data' in latest:
                # Fresh sample: the first message may predate the robot settling.
                self.imu_msg = None
                sub = self.create_subscription(
                    latest[f'/{s["name"]}/data'].__class__, f'/{s["name"]}/data',
                    lambda m: setattr(self, 'imu_msg', m), qos_profile_sensor_data)
                self.wait_for(lambda: self.imu_msg is not None, 10, 'imu')
                az = self.imu_msg.linear_acceleration.z
                r.add('imu: linear_acceleration.z at rest', '9.8 +-0.3', f'{az:.3f}', abs(az - 9.8) <= 0.3)
                self.destroy_subscription(sub)
            if s['type'] == 'gps' and f'/{s["name"]}/fix' in latest:
                fix = latest[f'/{s["name"]}/fix']
                near = abs(fix.latitude - 45.0) < 1e-3 and abs(fix.longitude - 9.0) < 1e-3
                r.add('gps: near world origin', '45.0, 9.0', f'{fix.latitude:.6f}, {fix.longitude:.6f}', near)

        # Self-hit: with no obstacle in the world a 2D scan must be empty.
        for s in self.cfg['sensors']:
            if s['type'] == 'lidar_2d':
                scan = self.fresh(LaserScan, f'/{s["name"]}/scan')
                finite = [x for x in scan.ranges if math.isfinite(x)]
                r.add(f'lidar_2d {s["name"]}: no self-hit', '0 finite returns',
                      f'{len(finite)} finite' + (f' (min {min(finite):.2f} m)' if finite else ''),
                      not finite)

        # TF
        for target in ['base_link'] + [s['name'] + '_link' for s in self.cfg['sensors']]:
            src = 'odom' if target == 'base_link' else 'base_link'
            ok = False
            end = time.time() + 15
            while time.time() < end and not ok:
                rclpy.spin_once(self, timeout_sec=0.1)
                ok = self.tf_buffer.can_transform(src, target, rclpy.time.Time())
            r.add(f'tf: {src} -> {target}', 'resolves', 'ok' if ok else 'missing', ok)

    def fresh(self, cls, topic, timeout=20):
        box = []
        sub = self.create_subscription(cls, topic, box.append, qos_profile_sensor_data)
        self.wait_for(lambda: len(box) > 0, timeout, topic)
        self.destroy_subscription(sub)
        return box[-1]

    def scan_at(self, topic, angle, n=5):
        """Median range at `angle` (rad) over n fresh scans."""
        vals = []
        box = []
        sub = self.create_subscription(LaserScan, topic, box.append, qos_profile_sensor_data)
        self.wait_for(lambda: len(box) >= n, 30, topic)
        self.destroy_subscription(sub)
        for scan in box[:n]:
            idx = int(round((angle - scan.angle_min) / scan.angle_increment))
            vals.append(scan.ranges[idx])
        return float(np.median(vals))

    def spawn_scenario(self, scenario):
        subprocess.run(['ros2', 'launch', 'rb_bringup', 'obstacles.launch.py', f'scenario:={scenario}'],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)
        self.spin_for_sim(3.0)

    def cloud_xyz(self, msg):
        offs = {f.name: f.offset for f in msg.fields}
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
        cols = [buf[:, offs[a]:offs[a] + 4].copy().view(np.float32).ravel() for a in 'xyz']
        pts = np.stack(cols, axis=1)
        return pts[np.isfinite(pts).all(axis=1)]

    def remove_entities(self, names):
        for name in names:
            subprocess.run(['ros2', 'run', 'ros_gz_sim', 'remove', '--ros-args', '-p', 'world:=flat',
                            '-p', f'entity_name:={name}'], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=60)
        self.spin_for_sim(1.0)

    def obstacle_checks(self):
        r = self.results
        self.spawn_scenario('test_box')
        for s in self.cfg['sensors']:
            if s['type'] == 'lidar_2d':
                rng = self.scan_at(f'/{s["name"]}/scan', 0.0)
                r.add(f'lidar_2d {s["name"]}: range at 0 deg', '2.5 +-0.1 m', f'{rng:.3f} m',
                      abs(rng - 2.5) <= 0.1)
            if s['type'] == 'rgbd':
                msg = self.fresh(PointCloud2, f'/{s["name"]}/points')
                pts = self.cloud_xyz(msg)
                if len(pts) == 0:
                    r.add(f'rgbd {s["name"]}: cloud', 'finite points', 'empty', False)
                    continue
                mx, mz = float(np.median(pts[:, 0])), float(np.median(pts[:, 2]))
                r.add(f'rgbd {s["name"]}: cloud frame', f'{s["name"]}_link', msg.header.frame_id,
                      msg.header.frame_id == f'{s["name"]}_link')
                r.add(f'rgbd {s["name"]}: cloud orientation', 'median x > 0 and x > |z|',
                      f'x={mx:.2f} z={mz:.2f}', mx > 0 and mx > abs(mz))

        # Every obstacle model loads; the cylinder is placed where the lidar can see it.
        others = [m for m in ALL_MODELS if m != 'cylinder_0p5']
        scenario = {'obstacles': [{'model': 'cylinder_0p5', 'name': 'check_cyl', 'x': 0.0, 'y': 2.0, 'yaw': 0.0}]
                    + [{'model': m, 'name': f'check_{m}', 'x': -4.0, 'y': -2.0 - 2.0 * i, 'yaw': 0.0}
                       for i, m in enumerate(others)]}
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
            yaml.safe_dump(scenario, f)
        self.spawn_scenario(f.name)
        os.unlink(f.name)
        for s in self.cfg['sensors']:
            if s['type'] == 'lidar_2d':
                rng = self.scan_at(f'/{s["name"]}/scan', math.pi / 2)
                r.add(f'obstacle cylinder_0p5: range at +90 deg', '1.75 +-0.1 m', f'{rng:.3f} m',
                      abs(rng - 1.75) <= 0.1)
                break
        out = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True, timeout=30).stdout
        for o in scenario['obstacles']:
            r.add(f'obstacle {o["model"]}: spawned', 'listed by gz', 'yes' if o['name'] in out else 'no',
                  o['name'] in out)
        # Clear the world so the motion checks start from an empty scene.
        self.remove_entities(['box_1'] + [o['name'] for o in scenario['obstacles']])
        out = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True, timeout=30).stdout
        left = [n for n in ['box_1'] + [o['name'] for o in scenario['obstacles']] if n in out]
        r.add('obstacles: removed again', 'none left', 'none' if not left else str(left), not left)


def assert_no_sim_running():
    """A second simulation in the same ROS domain makes every result meaningless."""
    rclpy.init()
    node = rclpy.create_node('rb_smoke_preflight')
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    busy = node.count_publishers('/clock') > 0 or node.count_publishers('/cmd_vel') + \
        node.count_subscribers('/cmd_vel') > 0
    node.destroy_node()
    rclpy.try_shutdown()
    if busy:
        sys.exit('A simulation is already running in this ROS domain (/clock or /cmd_vel is active). '
                 'Stop it first, or use --attach. Check for stray "gz sim" processes.')


def start_sim(robot, scenario):
    cmd = ['ros2', 'launch', 'rb_bringup', 'sim.launch.py', f'robot:={robot}', f'scenario:={scenario}',
           'gui:=false', 'rviz:=false', 'ground_truth:=true']
    log = open(os.path.join(tempfile.gettempdir(), f'rb_sim_{robot}.log'), 'w')
    print(f'sim log: {log.name}', flush=True)
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop_sim(proc):
    """Stop the whole process group. Never pkill by name: that can match the calling shell."""
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=20)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', default='default', help='robot config name or absolute path')
    ap.add_argument('--attach', action='store_true', help='test an already running sim instead of starting one')
    args = ap.parse_args()

    desc = get_package_share_directory('rb_description')
    path = args.robot if os.path.isabs(args.robot) else os.path.join(desc, 'config', 'robots', args.robot + '.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    drive = cfg['drive']
    print(f'== robot {args.robot} (drive {drive}, sensors {[s["name"] for s in cfg.get("sensors") or []]})', flush=True)

    results = Results()
    if not args.attach:
        assert_no_sim_running()
    proc = None if args.attach else start_sim(args.robot, 'none')
    rclpy.init()
    try:
        node = Tester(results, cfg)
        node.wait_ready()
        if cfg.get('sensors'):
            node.sensor_checks_rest()
        if args.robot == 'full_sensors':
            node.obstacle_checks()  # needs the robot at its spawn pose, so before moving
        node.motion_checks(drive)
        node.example_driver_check()
    except Exception as e:  # report instead of a bare traceback so the table still prints
        results.add('test run', 'completes', f'{type(e).__name__}: {e}', False)
    finally:
        if proc is not None:
            stop_sim(proc)
        rclpy.try_shutdown()
    ok = results.table()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
