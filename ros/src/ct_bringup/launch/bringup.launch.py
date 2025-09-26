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
    ros2_control_dir = get_package_share_directory('ct_control')  
    ps4_teleop_config = os.path.join(bringup_dir, 'config', 'ps4_config.yaml')
    mecanum_controller_yaml = os.path.join(bringup_dir, 'config', 'mecanum_drive_controllers.yaml')

    # Config files
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

    # Read URDF robot description
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # RPLIDAR launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'view_rplidar_a1_launch.py'),
        ),
        launch_arguments={'frame_id': 'taitc_lidar_link'}.items()
    )

    # Robot state publisher
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # SLAM lifecycle node   
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
    )

    # Lifecycle activation for SLAM
    activate_configure = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'configure'],
            output='screen'
        )]
    )

    activate_activate = TimerAction(
        period=5.0, 
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', 'slam_toolbox', 'activate'],
            output='screen'
        )]
    )

    # Teleop serial node (mecanum compatible)
    teleop_serial_node = Node(
        package='ct_bringup',
        executable='teleop_node',  
        name='teleop_node',
        output='screen'
    )

    # Joystick driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05, 'autorepeat_rate': 20.0}],
        output='screen'
    )

    # PS4 controller teleop -> cmd_vel
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[ps4_teleop_config], 
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    mecanum_controller_loader = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--param-file', mecanum_controller_yaml,
            '--set-state', 'active',
            'mecanum_drive_controller'
        ],
        output='screen'
    )

    return LaunchDescription([
        activate_configure,
        activate_activate,
        rplidar_launch,
        robot_state_node,       
        slam_node,
        teleop_serial_node,
        mecanum_controller_loader
    ])
