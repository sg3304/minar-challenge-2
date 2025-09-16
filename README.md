# TODO Reorganize
# Random Notes:
ros2 pkg create --build-type ament_python turtlesim_shapes --dependencies rclpy geometry_msgs turtlesim



colcon build
source install/setup.bash

both terminals have to be sourced from setup.bash:

ros2 run turtlesim_shapes turtle_shapes
ros2 run turtlesim turtlesim_node



extended kalman filter

static transform publisher etc

tf listener: rviz package

scan / tf3/ odom / cmd vel ->slam toolbox ->



PROJECT PLAN:

deadline:
SLAM ROBOT
requirements:
PROVIDED HARDWARE
ROS





# SETUP
https://index.ros.org/p/rplidar_ros/#jazzy


$ sudo apt install ros-humble-slam-toolbox

## workspace setup

mkdir -p ~/cha2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash

#libs
joystick package: https://index.ros.org/p/joy/
python prebuilt code: teleop_twist_joy
test with rostopic echo /cmd_vel
os2 topic echo /scan.

pkg creation:

ros2 pkg create my_robot_description --build-type ament_cmake
ros2 pkg create ct_bringup --build-type ament_python --dependencies rclpy launch
ros2 pkg create my_robot_teleop --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs
ros2 pkg create my_robot_sensors --build-type ament_python --dependencies rclpy sensor_msgs
ros2 pkg create my_robot_navigation --build-type ament_python --dependencies rclpy nav2_msgs geometry_msgs


THank you gpt:

Start Small

Write a publisher/subscriber node in Python.

Test publishing fake cmd_vel → verify motor controller responds.

Integrate Joystick

Install joy and teleop_twist_joy.

Add Lidar

Start lidar driver → check ros2 topic echo /scan.

Visualize in RViz.

Launch System Together

Write a bringup.launch.py that launches:

Joystick teleop

Base controller

Lidar

RViz

Iterate

Start with teleop control.

Add sensor fusion (e.g., IMU, wheel odometry).

Progress towards SLAM/nav2 if desired.

🔑 Key Concepts You’ll Need

ROS 2 Nodes & Topics (Python).

Launch system (Python launch files).

TF2: coordinate transforms (base_link, odom, laser).

URDF/Xacro: robot description.

RViz2: visualization.

Colcon: build system.

Joystick → cmd_vel → base controller pipeline.

👉 Suggested Learning Order:

ROS 2 basics (pub/sub, launch, params).

Joystick teleop.

cmd_vel → motor control.

Lidar integration.

RViz visualization.

(Optional) Gazebo simulation.

(Optional) SLAM + Navigation.