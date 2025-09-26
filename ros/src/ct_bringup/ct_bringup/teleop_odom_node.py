#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from math import sin, cos, pi
import numpy as np
import serial

# ------------------------
# Parameters
# ------------------------
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200


# ------------------------
# Teleop Node: cmd_vel → Arduino (wheel speeds)
# ------------------------
class TeleopOdomNode(Node):
    def __init__(self):
        super().__init__('teleop_serial_node')
        
        # Subscribe to velocity commands
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Serial connection to Arduino
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"Opened serial port {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # Robot parameters (adjust to your robot)
        self.length = 0.3         # front-back wheel separation [m]
        self.width = 0.25         # left-right wheel separation [m]
        self.wheel_radius = 0.05  # wheel radius [m]

    def cmd_callback(self, msg: Twist):
        """
        Convert cmd_vel (Twist) to mecanum wheel velocities
        and send them to Arduino as [w1,w2,w3,w4]\n
        """
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        R = self.wheel_radius
        L = self.length / 2.0
        W = self.width / 2.0
        LW = L + W

        # Mecanum inverse kinematics
        w1 = (vx - vy - LW * omega) / R  # front-left
        w2 = (vx + vy + LW * omega) / R  # front-right
        w3 = (vx + vy - LW * omega) / R  # rear-left
        w4 = (vx - vy + LW * omega) / R  # rear-right

        # Format and send to Arduino
        command = f"[{w1:.4f},{w2:.4f},{w3:.4f},{w4:.4f}]\n"
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().debug(f"Sent wheel velocities: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


# ------------------------
# Odom Node: fb_speed → odometry + TF
# ------------------------
class OdomNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # Pub, sub, bc
        self.speedSub = self.create_subscription(Twist, '/fb_speed', self.fb_speed_callback, 10)
        self.odomPub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcast = TransformBroadcaster(self)
        self.timer1 = self.create_timer(0.1, self.odom_update)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.omega_z = 0.0
        self.get_logger().info("Odometry node running (and broadcasting TF)")

    def fb_speed_callback(self, msg: Twist):
        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y
        self.omega_z = msg.angular.z

    def odom_update(self):
        dt = 0.1  # Time step

        delta_x = (self.vel_x * cos(self.theta) - self.vel_y * sin(self.theta)) * dt
        delta_y = (self.vel_y * cos(self.theta) + self.vel_x * sin(self.theta)) * dt
        delta_theta = self.omega_z * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = np.mod((self.theta + delta_theta), 2 * pi)  # keep in [0, 2pi)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = cos(self.theta / 2.0)

        odom_msg.twist.twist.linear.x = self.vel_x
        odom_msg.twist.twist.linear.y = self.vel_y
        odom_msg.twist.twist.angular.z = self.omega_z

        self.odomPub.publish(odom_msg)

        # Broadcast TF2
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = sin(self.theta / 2.0)
        transform.transform.rotation.w = cos(self.theta / 2.0)

        self.tf_broadcast.sendTransform(transform)


# ------------------------
# Main entry
# ------------------------
def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopOdomNode()
    odom_node = OdomNode()

    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(teleop_node)
        executor.add_node(odom_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.close()
        teleop_node.destroy_node()
        odom_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
