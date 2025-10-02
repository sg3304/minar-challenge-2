import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import numpy as np

# IN MM
RADIUS = (79/2)/1000
LX = (84.1)/1000
LY = (92.5)/1000

from std_msgs.msg import String

class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller_node")
        # String publisher for Arduino
        self.motorPublisher = self.create_publisher(String, '/controlspeed', 10)
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
        self.send_to_arduino(x_vel, y_vel, w_z)

    def cmdcallback(self, msg: Twist):
        x_vel = msg.linear.x
        y_vel = msg.linear.y
        w_z = msg.angular.z
        self.send_to_arduino(x_vel, y_vel, w_z)

    def send_to_arduino(self, x_vel, y_vel, w_z):
        smoothOpVel = np.array([[x_vel],[y_vel],[w_z]])
        w_speeds = self.inv_kin(smoothOpVel)
        # Format as Arduino-readable string
        arduino_msg = String()
        arduino_msg.data = f"[{w_speeds[0,0]:.2f},{w_speeds[1,0]:.2f},{w_speeds[2,0]:.2f},{w_speeds[3,0]:.2f}]"
        self.motorPublisher.publish(arduino_msg)

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


def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    rclpy.spin(motion_controller)
    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
