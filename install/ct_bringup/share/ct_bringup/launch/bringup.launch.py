from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    This function creates and returns the LaunchDescription object,
    which defines all the nodes and launch files to be started.
    """

    # Get package paths
    # These lines use a ROS 2 utility to find the install path of different packages.
    # This ensures that the launch file can find the necessary resources no matter where they are installed.
    bringup_dir = get_package_share_directory('ct_bringup')
    teleop_dir = get_package_share_directory('ct_teleop')
    sensors_dir = get_package_share_directory('ct_sensors')
    description_dir = get_package_share_directory('ct_description')

    # Paths to config files
    # We define the full paths to configuration files and other resources.
    # The 'os.path.join' function is used to create file paths in a way that works on all operating systems.
    teleop_config = os.path.join(teleop_dir, 'config', 'joystick.yaml')
    rviz_config = os.path.join(bringup_dir, 'config', 'robot.rviz')
    urdf_file = os.path.join(description_dir, 'urdf', 'ct.urdf')

    # Include another launch file
    # This is a powerful feature of ROS 2 launch files. It lets you include and run
    # other launch files, making your main launch file cleaner and more modular.
    # Here, we're including the launch file for the RPLIDAR sensor.
    rplidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rplidar_ros'),
                    'launch',
                    'view_rplidar_a1_launch.py'
                )
            )
        )

    # Load URDF into robot_state_publisher
    # The Universal Robot Description Format (URDF) file describes the robot's physical structure,
    # including its joints and links. We read the contents of this file into a variable.
    # The 'robot_state_publisher' node will use this information to broadcast the robot's state.
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # The LaunchDescription object contains a list of all actions to be performed.
    # These actions can be starting nodes, including other launch files, etc.
    return LaunchDescription([
        # Robot description publisher
        # The 'robot_state_publisher' node takes the robot's URDF description and
        # joint states and publishes the 3D transforms for all the robot's links.
        # This is essential for visualizing the robot in tools like RViz.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Joystick driver
        # The 'joy_node' is a standard ROS 2 package that reads input from a joystick
        # and publishes it on the '/joy' topic. This is a common way to control robots.
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),


        # Lidar driver
        # We include the launch file we defined earlier to start the RPLIDAR node.
        rplidar_launch,

        # RViz
        # This commented-out section shows how you would launch RViz, the ROS 2 visualization tool.
        # It's commented out because the 'view_rplidar_a1_launch.py' file already launches RViz.
        # It's a good practice to avoid launching duplicate nodes.
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config]
        # )
    ])