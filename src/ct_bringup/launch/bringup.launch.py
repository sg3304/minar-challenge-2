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

    # Config files
    slam_params_file = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

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

    # SLAM node
    slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': False}]
        )

    # Lifecycle activation (delay to allow lidar and odometry and stuff to start)
    #https://index.ros.org/p/lifecycle
    #https://github.com/ros2/demos/blob/jazzy/lifecycle/README.rst

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
    # Static transform: odom -> base-link   
    # These datapoints would come from the read odom node, which would read the instruments coming from the wheels.
    
    static_odom_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_odom_tf',  
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
)

    return LaunchDescription([
        rplidar_launch,
        robot_state_node,
       # static_lidar_tf,  
        activate_configure,
        activate_activate,
        static_odom_tf,
        slam_node
        
    ])
