import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import numpy as np
import serial
import time
# Serial parameters
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# IN MM
RADIUS = (79/2)/1000
LX = (84.1)/1000
LY = (92.5)/1000

class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller_node")
        # Serial connection
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Opened serial port {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # Publishers and subscribers
        self.motorPublisher = self.create_publisher(Float32MultiArray, '/controlspeed', 10)
        self.joySubscriber = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        self.cmdSubscriber = self.create_subscription(Twist, '/cmd_vel', self.cmdcallback, 10)
        self.feedbackSub = self.create_subscription(Float32MultiArray, '/fb_rot', self.fbCallback, 10)
        self.feedbackPub = self.create_publisher(Twist, '/fb_speed', 10)

        self.get_logger().info("Motion controller node has started!")

        self.feedbackMsg = Twist()
        self.invKinMatrix = np.array([
            [1,-1, -(LX+LY)],
            [1, 1,  (LX+LY)],
            [1, 1, -(LX+LY)],
            [1,-1,  (LX+LY)]
        ])
    def joycallback(self, msg: Joy):
        x_vel = 0.8 * msg.axes[1]  # m/s
        y_vel = 0.8 * msg.axes[0]
        w_z = 2 * msg.axes[3]      # rad/s
        smoothOpVel = np.array([[x_vel],[y_vel],[w_z]])
        w_speeds = self.inv_kin(smoothOpVel)
        self.publish_speeds(w_speeds)

    def cmdcallback(self, msg: Twist):
        x_vel = msg.linear.x
        y_vel = msg.linear.y
        w_z = msg.angular.z
        smoothOpVel = np.array([[x_vel],[y_vel],[w_z]])
        w_speeds = self.inv_kin(smoothOpVel)
        self.publish_speeds(w_speeds)

    def fbCallback(self, msg: Float32MultiArray):
        fl, fr, rl, rr = msg.data
        self.feedbackMsg.linear.x = (fl + fr + rl + rr) * (RADIUS / 4)
        self.feedbackMsg.linear.y = (-fl + fr + rl - rr) * (RADIUS / 4)
        self.feedbackMsg.angular.z = (-fl + fr - rl + rr) * (RADIUS / (4*(LX+LY)))
        self.feedbackPub.publish(self.feedbackMsg)

    def inv_kin(self, smoothOpVel):
        w_speeds = np.dot((self.invKinMatrix / RADIUS), smoothOpVel)
        maxOmega = np.max(np.abs(w_speeds))
        if maxOmega > 30:
            speedFactor = 30 / maxOmega
            w_speeds = w_speeds * speedFactor
        return w_speeds

    def publish_speeds(self, w_speeds):
        # Publish to ROS topic
        motor_msg = Float32MultiArray()
        motor_msg.data = w_speeds.flatten().tolist()
        self.motorPublisher.publish(motor_msg)

        command = f"[{w_speeds[0,0]:.4f},{w_speeds[1,0]:.4f},{w_speeds[2,0]:.4f},{w_speeds[3,0]:.4f}]\n"
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent to Arduino: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
    def read_serial_feedback(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('[') and line.endswith(']'):
                    parts = line.strip('[]').split(',')
                    if len(parts) == 4:
                        fl, fr, rl, rr = [float(x) for x in parts]

                        msg = Twist()
                        msg.linear.x = (fl + fr + rl + rr) * (RADIUS / 4)
                        msg.linear.y = (-fl + fr + rl - rr) * (RADIUS / 4)
                        msg.angular.z = (-fl + fr - rl + rr) * (RADIUS / (4*(LX+LY)))

                        self.feedbackPub.publish(msg)
                        self.get_logger().info(f"Published /fb_speed: {msg.linear.x:.2f}, {msg.angular.z:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error reading Arduino serial: {e}")
def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    try:
        rclpy.spin(motion_controller)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            motion_controller.ser.close()
        except Exception:
            pass
        motion_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()