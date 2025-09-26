from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    bringup_dir = get_package_share_directory('ct_bringup')
    description_dir = get_package_share_directory('ct_description')
    rplidar_dir = get_package_share_directory('rplidar_ros')

    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # RPLIDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'view_rplidar_a1_launch.py'),
        ),
        launch_arguments={'frame_id': 'taitc_lidar_link'}.items()
    )

    # State publisher
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # SLAM
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
    )

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

    # Joystick driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        output='screen'
    )

    # Teleop: joystick → Twist
    teleop_twist_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[os.path.join(
            get_package_share_directory('teleop_twist_joy'),
            'config/ps4.config.yaml'
        )],
        remappings=[('/cmd_vel', '/cmd_vel')]  # make sure matches your teleop_serial_node
    )

    # Your serial bridge node
    teleop_serial_node = Node(
        package='ct_bringup',
        executable='teleop_node',
        name='teleop_serial_node',
        output='screen'
    )

    return LaunchDescription([
        activate_configure,
        activate_activate,
        rplidar_launch,
        robot_state_node,
        slam_node,
        joy_node,
        teleop_twist_node,
        teleop_serial_node
    ])
