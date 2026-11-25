import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WEEK1_PKG = 'up_week1'
WEEK1_LAUNCH = 'week1.launch.py'


def _setup(context, *args, **kwargs):
    share = get_package_share_directory('up_week2')
    lighting = LaunchConfiguration('lighting').perform(context)
    use_mover = LaunchConfiguration('use_mover').perform(context).lower() == 'true'
    params = LaunchConfiguration('params_file').perform(context)

    if lighting not in ('normal', 'dim'):
        raise RuntimeError("lighting must be 'normal' or 'dim'")
    if not params:
        params = os.path.join(
            share, 'config',
            'detector.yaml' if lighting == 'normal' else 'detector_dim.yaml')
    world = os.path.join(
        share, 'worlds',
        'test_world_w2.sdf' if lighting == 'normal' else 'test_world_w2_dim.sdf')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(WEEK1_PKG), 'launch', WEEK1_LAUNCH)),
            launch_arguments={'world': world}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
             output='screen'),
        Node(package='up_week2', executable='red_target_detector',
             name='red_target_detector', parameters=[params], output='screen'),
    ]
    if use_mover:
        actions.append(Node(
            package='up_week2', executable='target_mover', name='target_mover',
            parameters=[os.path.join(share, 'config', 'target_path.yaml'),
                        {'use_sim_time': True}],
            output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lighting', default_value='normal',
                              description='normal | dim'),
        DeclareLaunchArgument('use_mover', default_value='true',
                              description='true | false'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='detector params (default: per lighting)'),
        OpaqueFunction(function=_setup),
    ])
