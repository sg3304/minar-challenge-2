from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get package paths
    bringup_dir = get_package_share_directory('ct_bringup')
    teleop_dir = get_package_share_directory('ct_teleop')
    sensors_dir = get_package_share_directory('ct_sensors')
    description_dir = get_package_share_directory('ct_description')

    # Paths to config files
    teleop_config = os.path.join(teleop_dir, 'config', 'joystick.yaml')
    rviz_config = os.path.join(bringup_dir, 'config', 'robot.rviz')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

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

        # Lidar driver (RPLidar A1)
        Node(
            package='rplidar_ros',
            executable='rplidarNode',
            name='rplidar',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('rplidar_ros'),
                'config',
                'rplidar.yaml'
            )]
        ),

        # Lidar driver
        Node(
            package='ct_sensors',
            executable='lidar_node',  # <-- your Python node
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
