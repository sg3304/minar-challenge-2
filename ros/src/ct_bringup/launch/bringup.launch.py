from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Package paths
    bringup_dir = get_package_share_directory('ct_bringup')
    description_dir = get_package_share_directory('ct_description')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    joy2twist_share = get_package_share_directory('ros2_joy_twist')

    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav_conf.yaml')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Launch RPLIDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'view_rplidar_a1_launch.py')
        ),
        launch_arguments={'frame_id': 'lidar_link'}.items()
    )

    # Robot state publisher
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'publish_frequency': 50.0}]
    )

    # Static TF from base_link to lidar_link
    # static_tf_base_to_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_base_to_laser',
    #     output='screen',
    #     arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar_link']
    # )

    # SLAM node (lifecycle)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
    )

    # Activate SLAM only after LIDAR + TF are ready
    activate_slam = TimerAction(
        period=10.0,  # wait 10 seconds for sensors/TF
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'configure'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'activate'],
                output='screen'
            )
        ]
    )

    # Odometry node
    odom_node = Node(
        package='ct_bringup',
        executable='odometry_node',  
        name='odometry_node',
        output='screen'
    )

    # Motion controller
    motion_controller_node = Node(
        package='ct_bringup',
        executable='motion_controller_node',  
        name='motion_controller_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud_rate': 115200
        }]
    )
    # rl_nav_node = Node(
    #     package='ct_navigation',   # your RL package
    #     executable='ppo_inference.py',
    #     name='navigation_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': False}]
    #)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'false'
        }.items()
    )
    return LaunchDescription([
        rplidar_launch,
        robot_state_node,
       # static_tf_base_to_laser,
        odom_node,
        motion_controller_node,
        slam_node,
        activate_slam,
        nav2_bringup
    ])

    # mappings = os.path.join(joy2twist_share, 'mappings.yaml')
    # joy2twist = Node(
    #     package='ros2_joy_twist',
    #     executable='joy_to_twist',
    #     name='joy_to_twist',
    #     output='screen',
    #     parameters=[mappings],
    # )
