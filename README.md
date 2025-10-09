# Challenge 2 SLAM Robot 🤖

## 🚀 Getting Started

This section outlines the step-by-step process for setting up the development environment, installing dependencies, and building the project.

### 1\. Workspace Setup

A **ROS 2 workspace** is where you'll store all your project packages. It's best practice to create a dedicated workspace for each project.

1.  **Create the workspace directory**:

    ```bash
    mkdir -p ~/{YOUR_WORKSPACE}/ros/src
    ```

    This command creates the `cha2_ws` directory, the `ros` directory and a `src` folder inside it, which is where you'll place your ROS 2 packages.

2.  **Navigate into the workspace**:    

    ```bash
    cd ~/cha2_ws
    ```

3.  **Build the workspace**:

    ```bash
    colcon build
    ```

    This command compiles any packages found in the `src` directory. You should run this command after adding new packages or making changes to your code.

4.  **Source the setup file**:

    ```bash
    source install/setup.bash
    ```

    This command makes the ROS 2 packages within your workspace available in your current terminal session.
    ⚠️ **Important:** You must run this command in every new terminal you open to work with ROS 2.

-----

## 🛠️ Installation & Dependencies

Before you can run the project, you need to install several key packages that the robot's code depends on.

### Core ROS 2 Packages

Install the necessary core packages using `apt`:

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

  * **`slam-toolbox`**: A powerful package for Simultaneous Localization and Mapping (SLAM).
  * **`joy`** and **`teleop-twist-joy`**: These packages work together to allow you to control the robot with a joystick.

### Lidar Driver

The project uses an RPLIDAR sensor for mapping and navigation. You'll need to install the ROS 2 driver from its GitHub repository.

1.  **Clone the driver repository**:

    ```bash
    cd ~/cha2_ws/src
    git clone https://github.com/Slamtec/rplidar_ros -b ros2
    ```

    The `-b ros2` flag ensures you clone the correct branch for ROS 2.

2.  **Build your workspace again** to include the new package:

    ```bash
    cd ~/cha2_ws
    colcon build
        source install/setup.bash
    ```

-----

## 🚀 Launching the Robot

Once everything is installed and built, you can launch the entire robot system with a single command. The `bringup.launch.py` file handles starting all the necessary nodes (lidar, joystick, motor controller, etc.).

From the root of your workspace (`~/cha2_ws`), run:

```bash
                ros2 launch ct_bringup bringup.launch.py
```
ros2 launch ct_bringup bringup.launch.py use_localization:=True
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
ros2 run nav2_map_server map_saver_cli
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate




This command will bring up the robot and its systems, allowing you to control it and begin SLAM.

-----

## 📚 Project Structure & Key Concepts

This project is organized into several ROS 2 packages, each with a distinct purpose.

  * **`ct_bringup`**: Contains the main launch file (`bringup.launch.py`) that orchestrates the startup of all other nodes in the system.
  * **`ct_description`**: Defines the physical properties of your robot using a **URDF** (Unified Robot Description Format) file. This is crucial for visualization in tools like **RViz**.
  * **`ct_teleop`**: Handles manual control of the robot by listening to joystick commands and publishing them to the `/cmd_vel` topic.
  * **`ct_sensors`**: Manages the data from the robot's hardware sensors, such as the Lidar and IMU.
  * **`ct_navigation`**: Integrates with the **Nav2** (Navigation 2) stack to enable autonomous navigation and SLAM.

### Fundamental Concepts

  * **ROS 2 Nodes & Topics**: The building blocks of any ROS system. Nodes are executable processes that communicate by sending messages over named topics.
  * **ROS 2 Launch System**: A powerful tool for starting multiple nodes and their parameters from a single Python script.
  * **TF2 (Transformations)**: Manages the coordinate frames of your robot, essential for tasks like navigation and sensor fusion.
  * **`colcon`**: The standard build tool for ROS 2.
  * **The `/cmd_vel` Pipeline**: A key pipeline in robotics where a command (from a joystick or navigation stack) publishes a **`geometry_msgs/Twist`** message to the `/cmd_vel` topic, which is then received by your robot's motor controller to move the wheels.