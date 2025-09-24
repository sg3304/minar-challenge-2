from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
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
    # Config files
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')
  #  diff_drive_params_file = os.path.join(ros2_control_dir, 'config', 'diff_drive_controllers.yaml')
    # URDF robot description
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # RPLIDAR launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'view_rplidar_a1_launch.py')
        )
    )

    # Robot state publisher
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Static transform: base_link -> lidar_link
    # https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1NodeOptions.html#_CPPv4N6rclcpp11NodeOptionsE
    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf',
        arguments=['0.04', '0', '0.08', '0', '0', '0', 'base_link', 'laser']
    )

    # SLAM lifecycle node   
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
        # automatically transition from configure -> activate 
        # remap or additional parameters here
    )


    # Lifecycle activation using TimerAction
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

    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    wheel_encoder_node = Node(
        package = 'ct_bringup',
        executable = 'wheel_encoder_node',  
        name ='wheel_encoder_node')
    
    teleop_node = Node(
        package = 'ct_bringup',
        executable = 'teleop_node',  
        name ='teleop_node')
    
    # diff_drive_controller_node = Node(
    #     package='diff_drive_controller',
    #     executable='diff_drive_controller',
    #     name='diff_drive_controller',
    #     output='screen',
    #     parameters=[diff_drive_params_file, {'use_sim_time': False}],
    #     remappings=[('/cmd_vel', '/cmd_vel')]
    # )
    return LaunchDescription([
        activate_configure,
        activate_activate,
        rplidar_launch,
        robot_state_node,
        static_lidar_tf,
        static_odom_tf,
        slam_node,
        wheel_encoder_node,
        teleop_node
      #  diff_drive_controller_node
    ])