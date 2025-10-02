import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_joy_twist')
    mappings = os.path.join(pkg_share, 'mappings.yaml')

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            # 'dev': '/dev/input/js0',
        }]
    )

    joy2twist = Node(
        package='ros2_joy_twist',
        executable='joy_to_twist',
        name='joy_to_twist',
        output='screen',
        parameters=[mappings],
        # remappings=[('cmd_vel', '/cmd_vel')]  # change if your robot uses a different topic
    )

    return LaunchDescription([joy, joy2twist])
