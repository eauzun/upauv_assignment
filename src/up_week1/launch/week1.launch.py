import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('up_week1'), 'worlds', 'test_world.sdf')

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        sigterm_timeout='5',
        sigkill_timeout='5',
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'],
        output='screen')

    camera_monitor = Node(
        package='up_week1',
        executable='camera_monitor',
        output='screen')

    heartbeat_publisher = Node(
        package='up_week1',
        executable='heartbeat_publisher',
        output='screen')

    heartbeat_subscriber = Node(
        package='up_week1',
        executable='heartbeat_subscriber',
        output='screen')

    return LaunchDescription([
        gazebo,
        bridge,
        camera_monitor,
        heartbeat_publisher,
        heartbeat_subscriber,
    ])
