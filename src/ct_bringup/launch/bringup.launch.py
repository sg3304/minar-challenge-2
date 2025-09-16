from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    bringup_dir = get_package_share_directory('my_robot_bringup')
    teleop_dir = get_package_share_directory('my_robot_teleop')
    sensors_dir = get_package_share_directory('my_robot_sensors')
    description_dir = get_package_share_directory('my_robot_description')

    # Paths to config files
    teleop_config = os.path.join(teleop_dir, 'config', 'joystick.yaml')
    rviz_config = os.path.join(bringup_dir, 'config', 'robot.rviz')
    urdf_file = os.path.join(description_dir, 'urdf', 'my_robot.urdf')

    # Load URDF into robot_state_publisher
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot description publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Teleop (joystick → cmd_vel)
        Node(
            package='my_robot_teleop',
            executable='teleop_node',   # <-- your Python node
            name='teleop_node',
            parameters=[teleop_config],
            output='screen'
        ),

        # Lidar driver (example node, replace with your lidar driver)
        Node(
            package='my_robot_sensors',
            executable='lidar_node',    # <-- your Python node
            name='lidar_node',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
