from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('ct_bringup')
    description_dir = get_package_share_directory('ct_description')
    rplidar_dir = get_package_share_directory('rplidar_ros')

    nav2_params = os.path.join(bringup_dir, 'config', 'nav_conf.yaml')
    slam_params = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'view_rplidar_a1_launch.py')
        ),
        launch_arguments={'frame_id': 'lidar_link'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    motion_controller = Node(
        package='ct_bringup',
        executable='motion_controller_node',
        output='screen',
        parameters=[{'port': '/dev/ttyACM0', 'baud_rate': 115200}]
    )

    odom_node = Node(
        package='ct_bringup',
        executable='odometry_node',
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[slam_params, {'use_sim_time': False}],
        output='screen'
    )

    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params-file': nav2_params
        }.items()
    )

    return LaunchDescription([
        rplidar_launch,
        robot_state_publisher,
        motion_controller,
        odom_node,
        slam_toolbox,
        nav2_bringup
    ])
