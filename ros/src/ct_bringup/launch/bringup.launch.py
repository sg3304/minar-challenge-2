from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # -----------------------------
    # Package directories
    # -----------------------------
    bringup_dir = get_package_share_directory('ct_bringup')
    description_dir = get_package_share_directory('ct_description')
    rplidar_dir = get_package_share_directory('rplidar_ros')

    # -----------------------------
    # Config and URDF files
    # -----------------------------
    nav2_params = os.path.join(bringup_dir, 'config', 'nav_conf.yaml')
    slam_params = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # -----------------------------
    # Launch arguments
    # -----------------------------
    launch_arguments = {
        'use_sim_time': 'False',
        'use_localization': 'False',  # Change to True to enable localization
        'params_file': nav2_params
    }

    map_path = os.path.join(bringup_dir, 'map', 'map.yaml')
    map_exists = os.path.exists(map_path)

    if map_exists:
        launch_arguments['map'] = map_path

    # -----------------------------
    # RPLIDAR launch
    # -----------------------------
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'view_rplidar_a1_launch.py')
        ),
        launch_arguments={'frame_id': 'lidar_link'}.items()
    )

    # -----------------------------
    # Core robot nodes
    # -----------------------------
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

    navigator = Node(
        package='ct_bringup',
        executable='click_to_nav_goal_node',
        name='click_to_nav_goal_node',
        output='screen'
    )

    # -----------------------------
    # SLAM node (only when not localizing)
    # -----------------------------
    use_localization = launch_arguments['use_localization'] == 'True'

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[slam_params, {'use_sim_time': False}],
        output='screen'
    )

    # -----------------------------
    # Nav2 launch (always included)
    # -----------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments=launch_arguments.items()
    )

    # -----------------------------
    # Assemble nodes
    # -----------------------------
    nodes = [
        rplidar_launch,
        robot_state_publisher,
        motion_controller,
        odom_node,
        navigator,
        nav2
    ]

    # Only include SLAM if NOT localizing
    if not use_localization:
        nodes.append(slam_toolbox)

    return LaunchDescription(nodes)
