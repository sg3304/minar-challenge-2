from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ROS Directory
    ros_dir = '/opt/ros/jazzy/share/'
    bringup_dir = get_package_share_directory('ct_bringup')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav_conf.yaml')

    # Custom Python nodes
    motor_controller = Node(
        package='robot',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
    )
    odom_pub = Node(
        package='robot',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
    )
    encoder_pub = Node(
        package='robot',
        executable='encoder_publisher',
        name='encoder_publisher',
        output='screen',
    )

    # Navigator
    navigator = Node(
        package='robot',
        executable='navigate_to_pose_client',
        name='navigate_to_pose_client',
        output='screen'
    )

    # Lidar node
    rplidar = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True
        }],
    )

    # Static transform base_link -> laser (adjust translation/rotation as needed)
    # static_tf_base_to_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_base_to_laser',
    #     output='screen',
    #     arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser']
    # )

    # nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_dir, 'nav2_bringup', 'launch', 'bringup_launch.py')
        ),
      #  launch_arguments={'use_sim_time': 'False', 'use_localization': 'True', 'map': f'{bringup_dir}/map/map.yaml', 'params-file': f'{bringup_dir}/config/nav2_params.yaml'}.items()
    )

    return LaunchDescription([
        motor_controller,
        odom_pub,
        encoder_pub,
        rplidar,
        navigator,
        static_tf_base_to_laser,
        nav2,
    ])
