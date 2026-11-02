"""Spawn the obstacles of a scenario into a running simulation.

    ros2 launch rb_bringup obstacles.launch.py scenario:=sparse
"""
import os

from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    world = LaunchConfiguration('world').perform(context)
    share = get_package_share_directory('rb_gazebo')
    path = scenario if os.path.isabs(scenario) else os.path.join(share, 'scenarios', scenario + '.yaml')
    if not os.path.isfile(path):
        names = sorted(f[:-5] for f in os.listdir(os.path.join(share, 'scenarios')) if f.endswith('.yaml'))
        raise RuntimeError(f"Scenario '{scenario}' not found ({path}). Available: {', '.join(names)}")
    with open(path) as f:
        obstacles = (yaml.safe_load(f) or {}).get('obstacles') or []

    actions = []
    for o in obstacles:
        model = os.path.join(share, 'models', o['model'], 'model.sdf')
        if not os.path.isfile(model):
            raise RuntimeError(f"{path}: unknown model '{o['model']}' (no {model})")
        actions.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-world', world, '-file', model, '-name', str(o['name']),
                       '-x', str(o.get('x', 0.0)), '-y', str(o.get('y', 0.0)),
                       '-z', str(o.get('z', 0.0)), '-Y', str(o.get('yaw', 0.0))]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', description='Name in rb_gazebo/scenarios or an absolute path'),
        DeclareLaunchArgument('world', default_value='flat'),
        OpaqueFunction(function=launch_setup),
    ])
