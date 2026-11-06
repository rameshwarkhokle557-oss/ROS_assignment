**Assessment submission (Rameshwar Khokle): see [README_ASSESSMENT.md](README_ASSESSMENT.md)**

# rb_sim

A small ROS 2 Jazzy + Gazebo Harmonic simulation of a wheeled robot. The robot is built from
primitive shapes and is meant to behave correctly, not to look realistic. The drive type and the
sensors are chosen by one YAML file, and obstacles can be added to a flat world.

| Package | Contents |
|---|---|
| `rb_description` | Robot xacro: drives, sensors, robot configs |
| `rb_control` | `ros2_control` controller configs and `cmd_vel_relay` |
| `rb_gazebo` | World, obstacle models, scenarios |
| `rb_bringup` | Launch files, sensor bridge templates, RViz config |
| `rb_assignment` | Starter package with an example driver |

The assignment itself is described in [ASSIGNMENT.md](ASSIGNMENT.md).

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy (`ros-jazzy-desktop`)
- Gazebo Harmonic, installed with the ROS packages below
- A GPU, or Mesa software rendering (see [Running headless](#running-headless))

## Install

```bash
sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
mkdir -p ~/ws/src && cd ~/ws/src
git clone <this repository> rb_sim
cd ~/ws
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

A devcontainer is provided in `.devcontainer/`. It is based on `osrf/ros:jazzy-desktop` and passes
`DISPLAY` and the X11 socket through, so the Gazebo GUI and RViz open on the host.

## Build

```bash
cd ~/ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch rb_bringup sim.launch.py robot:=default scenario:=none gui:=true rviz:=true ground_truth:=false
```

| Argument | Default | Meaning |
|---|---|---|
| `robot` | `default` | A name in `rb_description/config/robots/` (without `.yaml`) or an absolute path |
| `scenario` | `none` | A name in `rb_gazebo/scenarios/` or an absolute path; `none` spawns no obstacles |
| `gui` | `true` | `false` runs `gz sim -s --headless-rendering` |
| `rviz` | `true` | Open RViz |
| `ground_truth` | `false` | Publish the true pose on `/ground_truth/odom` |

Drive it from the keyboard in another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Running headless

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch rb_bringup sim.launch.py gui:=false rviz:=false
```

Set `LIBGL_ALWAYS_SOFTWARE=1` only when there is no GPU. On a machine with an NVIDIA GPU, forcing
software rendering made Gazebo crash in Mesa's EGL code, so leave it unset there. In the devcontainer,
install nothing extra: `libegl1` and `libgl1-mesa-dri` are already included.

## Topics

| Public topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Input. Mecanum also uses `linear.y` |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry |
| `/tf`, `/tf_static` | | `odom → base_link → sensors` |
| `/joint_states`, `/clock` | | |
| `/<sensor>/...` | | Per sensor type, see below |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | Only with `ground_truth:=true` |

`/cmd_vel` is a plain, unstamped `Twist`. Jazzy controllers only accept `TwistStamped`, so
`cmd_vel_relay` converts it, stamps it with simulation time, and clamps it to per-drive limits
from `rb_control/config/drives.yaml`.

| Sensor type | Topics |
|---|---|
| `lidar_2d` | `/<name>/scan` (`LaserScan`) |
| `lidar_3d` | `/<name>/points` (`PointCloud2`) |
| `rgb` | `/<name>/image`, `/<name>/camera_info` |
| `rgbd` | `/<name>/image`, `/<name>/depth_image`, `/<name>/camera_info`, `/<name>/points` |
| `imu` | `/<name>/data` (`Imu`) |
| `gps` | `/<name>/fix` (`NavSatFix`) |

Frames: `base_link` is the root, at ground level at the drive's kinematic centre. Each sensor has a
`<name>_link` frame; cameras also have `<name>_optical_frame`. Images and `camera_info` use the
optical frame. Point clouds use `<name>_link`.

## Drives

Set `drive:` in the robot config to one of:

| Drive | Notes |
|---|---|
| `diff_2w` | Two drive wheels and two passive casters |
| `skid_4w` | Four wheels. Turning relies on slip, so odometry position drifts on arcs |
| `ackermann` | Rear-wheel drive, front steering. It cannot turn on the spot: yaw is limited by the speed and a maximum steering angle, so a yaw command at zero speed does nothing |
| `mecanum` | Holonomic. It uses a simplified friction model (a friction direction per wheel, no rollers), so it is a good approximation, not an exact one |

Every drive goes through `gz_ros2_control`; there are no Gazebo drive plugins. The controller is
always named `drive_controller`.

### Swap the drive or sensors

Robot configs live in `rb_description/config/robots/`:

```yaml
drive: diff_2w
sensors:
  - {type: lidar_2d, name: front_lidar, mount: top_mount,   rate: 10}
  - {type: rgbd,     name: front_cam,   mount: front_mount, rate: 10}
  - {type: imu,      name: imu,         mount: top_mount,   rate: 100}
```

Copy one, edit it and launch with `robot:=<name>` or `robot:=/abs/path/to/file.yaml`. No other file
needs to change. Provided configs: `default`, `skid_lidar`, `ackermann_rgbd`, `mecanum_lidar`,
`full_sensors`.

Sensor keys:

- `type`: `lidar_2d`, `lidar_3d`, `rgb`, `rgbd`, `imu`, `gps`
- `name`: unique; it becomes the topic prefix and the frame name
- `mount`: `front_mount`, `top_mount` or `rear_mount` (rear is yawed 180°)
- `rate`: update rate in Hz
- optional `xyz` and `rpy`: offsets from the default sensor position, e.g. `xyz: [0, 0.1, 0]`
- optional `noise_stddev` on lidars (default 0)

The launch file stops with a message for an unknown drive, sensor type or mount, or a duplicate
sensor name. The lidar sees every visual in the world, including other sensor bodies, so keep a
lidar above other sensors on the same mount.

### Add a sensor

1. Add `rb_description/urdf/sensors/<type>.xacro` with a macro `sensor_<type>` taking at least
   `name mount rate`. Copy an existing file.
2. Include it in `rb_description/urdf/robot.urdf.xacro`.
3. Add `rb_bringup/config/bridge/<type>.yaml`, a list of bridge entries using `{name}` as the sensor
   name placeholder.

### Add a drive

1. Add `rb_description/urdf/drives/<drive>.xacro` defining a macro `drive` (chassis, wheels and a
   `ros2_control` block).
2. Add `rb_control/config/<drive>.yaml` with a controller named `drive_controller`.
3. Add an entry to `rb_control/config/drives.yaml`: the controller input topic, the spawner
   remaps and the relay limits.

## Obstacles

Obstacle models are static primitives in `rb_gazebo/models/`: `box_0p5`, `box_1m`, `cylinder_0p5`,
`wall_2m`, `wall_4m`, `barrier`, `traffic_cone`.

- **From a scenario at launch:** `scenario:=sparse` (also `corridor`, `slalom`, `test_box`).
- **Into a running simulation:**
  `ros2 launch rb_bringup obstacles.launch.py scenario:=slalom`
- **From the GUI:** the models appear in the Resource Spawner (the package exports
  `GZ_SIM_RESOURCE_PATH`). Source `install/setup.bash` before starting Gazebo.
- **New obstacle:** add `rb_gazebo/models/<name>/model.config` and `model.sdf`. The model needs a
  `<visual>`, because lidar and cameras do not see `<collision>`.
- **New scenario:** add `rb_gazebo/scenarios/<name>.yaml`:

```yaml
obstacles:
  - {model: box_1m, name: box_1, x: 3.0, y: 0.0, yaw: 0.0}
```

Rebuild (`colcon build`) after adding files so that they are installed.

## Tests

```bash
scripts/run_all_tests.sh               # every robot config, then config validation
scripts/smoke_test.py --robot default  # one config
```

Each test starts a headless simulation with ground truth, drives the robot and compares
the measured motion with `/ground_truth/odom`. `full_sensors` also checks every sensor and obstacle
model.

## Worked example: a six-wheel skid-steer drive (`skid_6w`)

This is not provided; it shows what adding a drive takes. Using the steps above:

1. Copy `skid_4w.xacro` and add a middle wheel pair at `x="0"`, using the same `wheel` macro. Add the
   two new joints to the `ros2_control` block.
2. Copy `skid_4w.yaml` and list three joints per side in `left_wheel_names` and `right_wheel_names`.
3. Add a `skid_6w` entry to `drives.yaml`, the same as `skid_4w`.
4. Re-measure `wheel_separation_multiplier`. Command a spin of 1.0 rad/s for 3 s and compare the true
   yaw from `/ground_truth/odom` with 3.0 rad. Scale the multiplier by the ratio of the
   commanded to the measured yaw until they agree. The 1.34 in `skid_4w.yaml` is specific to
   that wheel layout.

## Licence

Apache-2.0
