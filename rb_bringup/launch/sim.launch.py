"""Start the simulation: Gazebo, robot description, bridge, controllers, relay, obstacles, RViz.

    ros2 launch rb_bringup sim.launch.py robot:=default scenario:=none gui:=true rviz:=true ground_truth:=false
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MOUNTS = ('front_mount', 'top_mount', 'rear_mount')
WORLD = 'flat'


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def resolve_robot(robot, desc_share):
    """Return the robot config path for a name in config/robots/ or an absolute path."""
    if os.path.isabs(robot):
        path = robot
    else:
        path = os.path.join(desc_share, 'config', 'robots', robot + '.yaml')
    if not os.path.isfile(path):
        names = sorted(f[:-5] for f in os.listdir(os.path.join(desc_share, 'config', 'robots'))
                       if f.endswith('.yaml'))
        raise RuntimeError(f"Robot config '{robot}' not found ({path}). Available: {', '.join(names)}")
    return path


def validate(cfg, path, drives, sensor_types):
    """Fail early with a readable message instead of a xacro traceback."""
    if not isinstance(cfg, dict) or 'drive' not in cfg:
        raise RuntimeError(f'{path}: missing key "drive"')
    if cfg['drive'] not in drives:
        raise RuntimeError(f"{path}: unknown drive '{cfg['drive']}'. Known drives: {', '.join(sorted(drives))}")
    names = set()
    for i, s in enumerate(cfg.get('sensors') or []):
        for key in ('type', 'name', 'mount'):
            if key not in s:
                raise RuntimeError(f'{path}: sensor #{i} is missing "{key}"')
        if s['type'] not in sensor_types:
            raise RuntimeError(f"{path}: sensor '{s['name']}' has unknown type '{s['type']}'. "
                               f"Known types: {', '.join(sorted(sensor_types))}")
        if s['mount'] not in MOUNTS:
            raise RuntimeError(f"{path}: sensor '{s['name']}' has unknown mount '{s['mount']}'. "
                               f"Known mounts: {', '.join(MOUNTS)}")
        if s['name'] in names:
            raise RuntimeError(f"{path}: duplicate sensor name '{s['name']}'")
        names.add(s['name'])


def write_bridge_config(cfg, bridge_dir, ground_truth):
    entries = [{
        'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
        'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock',
        'direction': 'GZ_TO_ROS',
    }]
    if ground_truth:
        entries.append({
            'ros_topic_name': '/ground_truth/odom', 'gz_topic_name': '/ground_truth/odom',
            'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry',
            'direction': 'GZ_TO_ROS',
        })
    for s in cfg.get('sensors') or []:
        with open(os.path.join(bridge_dir, s['type'] + '.yaml')) as f:
            template = f.read().replace('{name}', s['name'])
        entries.extend(yaml.safe_load(template))
    fd, path = tempfile.mkstemp(prefix='rb_bridge_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(entries, f)
    return path


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    scenario = LaunchConfiguration('scenario').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    ground_truth = LaunchConfiguration('ground_truth').perform(context).lower() == 'true'

    desc_share = get_package_share_directory('rb_description')
    control_share = get_package_share_directory('rb_control')
    gazebo_share = get_package_share_directory('rb_gazebo')
    bringup_share = get_package_share_directory('rb_bringup')
    bridge_dir = os.path.join(bringup_share, 'config', 'bridge')

    drives = load_yaml(os.path.join(control_share, 'config', 'drives.yaml'))
    sensor_types = sorted(f[:-5] for f in os.listdir(bridge_dir) if f.endswith('.yaml'))
    config_path = resolve_robot(robot, desc_share)
    cfg = load_yaml(config_path)
    validate(cfg, config_path, drives, sensor_types)
    drive = drives[cfg['drive']]

    robot_description = xacro.process_file(
        os.path.join(desc_share, 'urdf', 'robot.urdf.xacro'),
        mappings={'config': config_path, 'ground_truth': str(ground_truth).lower()},
    ).toxml()
    bridge_config = write_bridge_config(cfg, bridge_dir, ground_truth)
    controller_yaml = os.path.join(control_share, 'config', cfg['drive'] + '.yaml')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    gz_args = '-r -v2 ' + os.path.join(gazebo_share, 'worlds', WORLD + '.sdf')
    if not gui:
        gz_args = '-s --headless-rendering ' + gz_args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['--ros-args', '-p', 'config_file:=' + bridge_config, '-p', 'use_sim_time:=true'])

    create = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', WORLD, '-topic', 'robot_description', '-name', 'rb', '-z', '0.01'])

    jsb = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'])
    drive_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['drive_controller', '--param-file', controller_yaml,
                   '--controller-manager-timeout', '60',
                   '--controller-ros-args', drive['controller_ros_args']])

    relay_params = {'use_sim_time': True, 'out_topic': drive['input_topic']}
    relay_params.update(drive['relay'])
    relay = Node(package='rb_control', executable='cmd_vel_relay.py', output='screen',
                 parameters=[relay_params])

    actions = [rsp, gazebo, bridge, create]
    actions.append(RegisterEventHandler(OnProcessExit(target_action=create, on_exit=[jsb])))
    actions.append(RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[drive_spawner, relay])))

    if scenario.lower() != 'none':
        obstacles = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'obstacles.launch.py')),
            launch_arguments={'scenario': scenario, 'world': WORLD}.items())
        actions.append(RegisterEventHandler(OnProcessExit(target_action=create, on_exit=[obstacles])))

    actions.append(Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'default.rviz')],
        parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='default',
                              description='Name in rb_description/config/robots or an absolute path'),
        DeclareLaunchArgument('scenario', default_value='none',
                              description='Name in rb_gazebo/scenarios, an absolute path, or none'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI; false runs headless'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ground_truth', default_value='false',
                              description='Publish /ground_truth/odom from the simulator'),
        OpaqueFunction(function=launch_setup),
    ])
