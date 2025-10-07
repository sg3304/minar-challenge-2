from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

import os


def generate_launch_description():
    # -------------------------------
    # Path setup
    # -------------------------------
    bringup_dir = get_package_share_directory('ct_bringup')
    description_dir = get_package_share_directory('ct_description')
    rplidar_dir = get_package_share_directory('rplidar_ros')

    # Config files
    slam_params = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav_conf.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # -------------------------------
    # Nodes
    # -------------------------------

    # RPLIDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'view_rplidar_a1_launch.py')
        ),
        launch_arguments={'frame_id': 'lidar_link'}.items()
    )

    # Robot State Publisher (TF from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'publish_frequency': 50.0}]
    )

    # Odometry node (integrates velocity)
    odom_node = Node(
        package='ct_bringup',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    # Motion controller (serial communication)
    motion_controller = Node(
        package='ct_bringup',
        executable='motion_controller_node',
        name='motion_controller_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud_rate': 115200
        }]
    )

    # SLAM Toolbox (builds /map and map→odom transform)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False, 'autostart': True}]
    )

    # Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Nav2 Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    # Nav2 Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator'
            ]
        }]
    )
    recoveries_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params]
    )

   

    # -------------------------------
    # Launch all components
    # -------------------------------
    return LaunchDescription([
        rplidar_launch,
        robot_state_publisher,
        motion_controller,
        odom_node,
        slam_toolbox,
        planner_server,
        controller_server,
        bt_navigator,
       # lifecycle_manager,
        recoveries_server_node

    ])

