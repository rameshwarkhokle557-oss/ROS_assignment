# Robotics Engineer Intern Assessment - Rameshwar Khokle

Work is on branch `rameshwar-assessment` of this fork. The original Accurate Controls repository is untouched.
The base README of the framework is `README.md`. All test evidence is in `evidence/`.

## 1. Environment

| | |
|---|---|
| OS | Ubuntu 24.04 under WSL2 on Windows (the only difference from the assessment setup) |
| ROS / Gazebo | ROS 2 Jazzy, Gazebo Harmonic 8.15.0 |
| Language | Python (ament_python package `rb_assignment`) |

On WSL2 the simulation ran at about 55-75 % real time, so measured topic rates are lower than the configured
ones (the 10 Hz LiDAR measured about 5.3 Hz). Timed tests therefore use the simulation clock.

## 2. Build

    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    git clone https://github.com/rameshwarkhokle557-oss/ROS_assignment.git
    cd ROS_assignment && git checkout rameshwar-assessment
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash

## 3. Launch

    ros2 launch rb_bringup sim.launch.py robot:=<config> scenario:=<scenario> ground_truth:=true

Run every other command in a second terminal after `source ~/ros2_ws/install/setup.bash`.
Stop nodes with Ctrl+C. Do not run two nodes that publish `/cmd_vel` at the same time.

## 4. Topics

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | input; relayed to `/drive_controller/cmd_vel` as TwistStamped |
| `/odom` | nav_msgs/Odometry | from the diff-drive controller |
| `/ground_truth/odom` | nav_msgs/Odometry | simulator pose, only with `ground_truth:=true` |
| `/front_lidar/scan` | sensor_msgs/LaserScan | 360 rays, 0.1-12 m |
| `/rear_lidar/scan` | sensor_msgs/LaserScan | config `dual_lidar_imu` |
| `/imu/data` | sensor_msgs/Imu | frame `imu_link` |
| `/front_cam/image`, `/front_cam/camera_info` | sensor_msgs | config `rgb_camera` |
| `/nearest_obstacle` | std_msgs/Float32MultiArray | `[distance_m, bearing_rad]`, bearing 0 = ahead, left positive |

## 5. Tasks

### S1 - Resize the robot
Files changed: `rb_description/urdf/common.xacro` (`wheel_r` 0.10 -> 0.12 m, `body_l` 0.60 -> 0.80 m),
`rb_control/config/diff_2w.yaml` and `skid_4w.yaml` (`wheel_radius` 0.10 -> 0.12).
These are test values chosen by me; the reviewer's target dimensions were not available to me. The model derives
visuals, collisions, inertia, mounts and caster height from the properties, so a new size needs one edit in
`common.xacro` plus the matching `wheel_radius` in the YAML (the controller does not read the Xacro).

Run: `ros2 launch rb_bringup sim.launch.py ground_truth:=true`, then drive with
`timeout 8 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`
(turn test: `"{angular: {z: 0.5}}"`).

Result: straight line `/odom` 0.610 m vs ground truth 0.609 m; turn `/odom` 63.64 deg vs ground truth 63.66 deg.
Evidence: `evidence/s1_straight.txt`, `evidence/s1_turn.txt`.

### S2 - Sensor configurations
- `rb_description/config/robots/dual_lidar_imu.yaml`: front LiDAR on `top_mount`, second LiDAR `rear_lidar` on
  `rear_mount` (that mount is rotated by pi, so it faces backwards), IMU.
- `rb_description/config/robots/rgb_camera.yaml`: RGB camera `front_cam` (type `rgb`) in place of the RGB-D camera.

Run: `ros2 launch rb_bringup sim.launch.py robot:=dual_lidar_imu` (or `robot:=rgb_camera`), then
`ros2 topic list`, `ros2 topic echo <scan or image topic> --once --field header`, `ros2 run tf2_tools view_frames`.
Frames: `rear_lidar_link`, `front_lidar_link`, `imu_link`, `front_cam_optical_frame`. IMU measured at 81-90 Hz
(100 Hz configured, simulation slower than real time).
TF: all frames form one tree (`odom -> base_link -> chassis -> mounts -> sensor links`); `rear_lidar_link` hangs
under `rear_mount`, `front_cam_link -> front_cam_optical_frame` under `front_mount`.
Evidence: `evidence/s0_baseline_tf.pdf` (default robot), `evidence/s2_config_a.txt`, `evidence/s2_config_a_tf.pdf`,
`evidence/s2_config_b.txt`, `evidence/s2_config_b_tf.pdf`.

### B1 - Nearest obstacle
Node: `rb_assignment/rb_assignment/nearest_obstacle.py`. It ignores `inf`, `NaN` and values outside
`[range_min, range_max]`, takes the smallest remaining range, and computes the bearing as
`angle_min + index * angle_increment`. Parameter: `scan_topic` (default `/front_lidar/scan`).

Run: launch with `scenario:=test_box`, then `ros2 run rb_assignment nearest_obstacle` and
`ros2 topic echo /nearest_obstacle`. Move the robot with `timeout 6 ros2 run rb_assignment example_driver`.
Result: 2.497 m at -0.5 deg before, 1.705 m at +0.5 deg after moving about 0.8 m (the box face is at x = 2.5 m;
the bearing sign flips because the two rays at +-0.5 deg hit the flat face at almost the same range).
Evidence: `evidence/b1_before.txt`, `evidence/b1_after.txt`.

### B3 - Emergency stop
Node: `rb_assignment/rb_assignment/emergency_stop.py`. It drives forward at 0.2 m/s and stops once the gap between
the chassis front and the obstacle is at most `stop_gap` (0.45 m). The LiDAR sits at the chassis centre, so the
stop range seen by the LiDAR is `front_offset + stop_gap = 0.40 + 0.45 = 0.85 m` (`front_offset` = `body_l / 2`).
Parameters: `speed`, `stop_gap`, `front_offset`, `window_deg` (20), `scan_timeout` (1.0 s), `max_nan_fraction` (0.5).

Safety logic:
- only rays within +-20 deg of straight ahead are checked (side walls are ignored)
- `+inf` (no echo): nothing on that ray; `NaN`: ignored but counted; more than 50 % NaN in the window: hold still
- `-inf` or below `range_min`: obstacle closer than the sensor can measure, treated as touching: stop
- no scan for more than 1 s, or no scan yet: hold still (zero velocity is published)
- once stopped the node stays stopped (latched)

Run: launch with `scenario:=test_box ground_truth:=true`, then `ros2 run rb_assignment emergency_stop`.
Change the gap live: `ros2 run rb_assignment emergency_stop --ros-args -p stop_gap:=0.35`.
Result: stopped at LiDAR range 0.85 m; ground-truth x = 1.689 m, so the true gap is
2.5 - (1.689 + 0.40) = 0.41 m, inside the required 0.3-0.6 m, no collision. Evidence: `evidence/b3_result.txt`.

### B2 - Square: open loop vs closed loop
Node: `rb_assignment/rb_assignment/drive_square.py`, 1 m sides, 0.2 m/s, 0.5 rad/s turns.
Launch WITHOUT a scenario (a box would be in the way), with `ground_truth:=true`, then:

    ros2 run rb_assignment drive_square --ros-args -p use_sim_time:=true -p mode:=open
    ros2 run rb_assignment drive_square --ros-args -p use_sim_time:=true -p mode:=closed
    python3 evidence/plot_b2.py

Open loop: timed forward and timed 90 deg turns. Closed loop: forward until the `/odom` distance reaches 1 m,
turn until the yaw error is below 0.01 rad, with a small heading correction while driving.

| Mode | `/odom` closing error | Ground-truth closing error |
|---|---|---|
| open | 2.9 cm, -0.3 deg | 2.8 cm, -0.9 deg |
| closed | 0.0 cm, -0.6 deg | 0.0 cm, -0.6 deg |

Closed loop corrects every segment from feedback; open loop lets small errors accumulate. One run per mode.
In closed loop `/odom` is the feedback source, so only ground truth shows the real error. Gazebo models little
wheel slip, so open-loop error is smaller than it would be on real hardware.
Evidence: `evidence/b2_open.txt`, `evidence/b2_closed.txt`, the two CSV files and `evidence/b2_trajectory.png`.

### S3 (bonus) - Parking area scenario
File: `rb_gazebo/scenarios/parking_area.yaml`. Three 2 m x 2 m bays to the left of a driving lane along +x.
Dividers are `wall_2m` rotated by 90 deg (`yaw: 1.5708`) at x = 2, 4, 6, 8; back wall at y = 3.0 (`wall_4m` at
x = 4, `wall_2m` at x = 7); parked cars (`box_1m`) in bays 1 and 3 at (3, 2) and (7, 2); a cone in the free bay 2;
a `wall_2m` across the lane end at (10, 0).

Run: `ros2 launch rb_bringup sim.launch.py scenario:=parking_area ground_truth:=true`, then
`gz model --list` and `ros2 run rb_assignment nearest_obstacle`.
Result: nearest obstacle 2.16 m at +28.6 deg (front face of `divider_1`). The layout without wall thickness would
give 2.24 m at +26.6 deg; the measured point (x about 1.90, y about 1.04) suggests a wall thickness of about 0.2 m
(inferred, not measured). An unrotated divider would give about 63 deg, so the 90 deg rotation works.
Evidence: `evidence/s3_models.txt`, `evidence/s3_nearest.txt`.

## 6. AI assistance
I used an AI assistant (Claude) for guidance, for drafting code and for explanations while working on this
assessment. I tested everything in the simulation, and I can explain the code and the configuration changes.

## 7. Known limitations
- The S1 dimensions are test values (see S1).
- The simulation ran at about 55-75 % real time on WSL2.
- B2 has one run per mode; no statistics.
- Wall thickness in the parking scenario was inferred from the LiDAR reading, not measured directly.
