from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package paths
    bringup_dir = get_package_share_directory('ct_bringup')
    description_dir = get_package_share_directory('ct_description')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    joy2twist_share = get_package_share_directory('ros2_joy_twist')

    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'view_rplidar_a1_launch.py'),
        ),
        launch_arguments={'frame_id': 'lidar_link'}.items()
    )

    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
    )

    activate_configure = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'configure'],
            output='screen'
        )]
    # )
    # static_tf_base_to_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_base_to_laser',
    #     output='screen',
    #     arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser']
    # )
    )

    activate_activate = TimerAction(
        period=5.0, 
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'activate'],
            output='screen'
        )]
    )

    
    odom_node = Node(
        package='ct_bringup',
        executable='odometry_node',  
        name='odometry_node',
        output='screen'
    )

    motion_controller_node = Node(
        package='ct_bringup',
        executable='motion_controller_node',  
        name='motion_controller_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',  # Arduino serial port
            'baud_rate': 115200      # Match Arduino sketch
        }]
    )

    # mappings = os.path.join(joy2twist_share, 'mappings.yaml')
    # joy2twist = Node(
    #     package='ros2_joy_twist',
    #     executable='joy_to_twist',
    #     name='joy_to_twist',
    #     output='screen',
    #     parameters=[mappings],
    # )

    return LaunchDescription([
        activate_configure,
        activate_activate,
        rplidar_launch,
        robot_state_node,       
        slam_node,
        motion_controller_node,
        odom_node,
      #  joy2twist
        # static_tf_base_to_laser
    ])
