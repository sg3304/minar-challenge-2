#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import serial
import math
import tf_transformations
from tf2_ros import TransformBroadcaster

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__('teleop_serial_node')
        
        # ROS interfaces
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, 'wheel_velocities', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Serial
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self.timer = self.create_timer(0.05, self.read_arduino)  # 20 Hz
        self.last_time = self.get_clock().now()

        # Robot params (set to your real values)
        self.length = 0.3       # front-back distance between wheel centers [m]
        self.width = 0.25       # left-right distance between wheel centers [m]
        self.wheel_radius = 0.05  # wheel radius [m]

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cmd_callback(self, msg: Twist):
        """
        Convert Twist (vx, vy, omega) into 4 mecanum wheel angular velocities (rad/s)
        and send to Arduino in the format: [w1,w2,w3,w4]\n
        where w1=front-left, w2=front-right, w3=rear-left, w4=rear-right
        """
        vx = msg.linear.x    # forward
        vy = msg.linear.y    # strafe (m/s)
        omega = msg.angular.z  # rotation about z (rad/s)

        R = self.wheel_radius
        L = self.length / 2.0
        W = self.width / 2.0
        LW = (L + W)

        # Inverse kinematics for mecanum (vx, vy, omega) -> wheel angular velocities
        # w = 1/R * [ ... ] (rad/s)
        w1 = (vx - vy - LW * omega) / R  # front-left
        w2 = (vx + vy + LW * omega) / R  # front-right
        w3 = (vx + vy - LW * omega) / R  # rear-left
        w4 = (vx - vy + LW * omega) / R  # rear-right

        command = f"[{w1:.4f},{w2:.4f},{w3:.4f},{w4:.4f}]\n"
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent to Arduino: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed sending to serial: {e}")

    def read_arduino(self):
        """
        Read a line from Arduino and expect a bracketed list: [w1,w2,w3,w4]
        where values are wheel angular velocities in rad/s. Compute vx, vy, omega
        using forward kinematics and update odometry.
        """
        try:
            if self.ser.in_waiting:
                raw = self.ser.readline().decode('utf-8').strip()
                if not raw:
                    return
                # allow stray characters; extract inside brackets if present
                line = raw.strip()
                if line.startswith('[') and line.endswith(']'):
                    line = line[1:-1]
                parts = [p for p in line.split(',') if p != '']
                if len(parts) != 4:
                    self.get_logger().warn(f"Unexpected wheel count from Arduino: {raw}")
                    return
                # parse floats
                w = [float(p) for p in parts]  # w1, w2, w3, w4 (rad/s)
                msg = Float32MultiArray()
                msg.data = w
                self.pub_wheels.publish(msg)

                # Forward kinematics: wheels -> vx, vy, omega
                R = self.wheel_radius
                L = self.length / 2.0
                W = self.width / 2.0
                LW = (L + W)

                w1, w2, w3, w4 = w

                vx = (R / 4.0) * (w1 + w2 + w3 + w4)
                vy = (R / 4.0) * (-w1 + w2 + w3 - w4)
                # omega uses LW term
                omega = (R / (4.0 * LW)) * (-w1 + w2 - w3 + w4)

                # Update odometry with vx, vy, omega
                self.update_odometry(vx, vy, omega)
        except Exception as e:
            self.get_logger().error(f"Error reading from Arduino: {e}")

    def update_odometry(self, vx, vy, omega):
        """
        Integrate pose using velocities in the robot frame.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        # handle very small / negative dt edgecases
        if dt <= 0.0:
            dt = 1e-6
        self.last_time = current_time

        # Convert robot-frame velocities to world-frame for integration
        # vx, vy are in robot frame; rotate by theta to get world velocities
        vwx = vx * math.cos(self.theta) - vy * math.sin(self.theta)
        vwy = vx * math.sin(self.theta) + vy * math.cos(self.theta)

        self.x += vwx * dt
        self.y += vwy * dt
        self.theta += omega * dt

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        # Create quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        self.pub_odom.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
