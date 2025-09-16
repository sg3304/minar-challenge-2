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