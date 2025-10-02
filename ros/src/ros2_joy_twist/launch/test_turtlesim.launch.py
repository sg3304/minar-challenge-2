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
    )

    # Your mapper; publishes to /cmd_vel (default)
    joy2twist = Node(
        package='ros2_joy_twist',
        executable='joy_to_twist',
        name='joy_to_twist',
        output='screen',
        parameters=[mappings],
        remappings=[
            ('cmd_vel', '/turtle1/cmd_vel'),  # turtlesim listens here
        ]
    )

    turtle = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    return LaunchDescription([joy, joy2twist, turtle])
