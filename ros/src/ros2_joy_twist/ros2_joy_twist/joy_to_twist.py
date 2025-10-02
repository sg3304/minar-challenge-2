#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyToTwist(Node):
    def __init__(self):
        super().__init__('joy_to_twist')

        # Declare params using your dotted keys
        self.declare_parameters('', [
            ('axis_linear.x', 1),
            ('axis_linear.y', 0),
            ('axis_linear_dpad.x', -1),
            ('axis_linear_dpad.y', -1),
            ('axis_angular.yaw', 3),
            ('scale_linear.x', 1.0),
            ('scale_linear.y', 1.0),
            ('scale_angular.yaw', 1.5),
            ('enable_button', 5),
            ('enable_turbo_button', 4),
            ('turbo_scale', 2.0),
            ('deadzone', 0.05),
        ])

        # Read params (ROS 2 allows dots in parameter names)
        gp = self.get_parameter
        self.ax_lin_x = gp('axis_linear.x').value
        self.ax_lin_y = gp('axis_linear.y').value
        self.ax_lin_dpad_x = gp('axis_linear_dpad.x').value
        self.ax_lin_dpad_y = gp('axis_linear_dpad.y').value
        self.ax_yaw = gp('axis_angular.yaw').value

        self.s_lin_x = gp('scale_linear.x').value
        self.s_lin_y = gp('scale_linear.y').value
        self.s_yaw = gp('scale_angular.yaw').value

        self.enable_btn = gp('enable_button').value
        self.enable_turbo_btn = gp('enable_turbo_button').value
        self.turbo_scale = gp('turbo_scale').value

        self.deadzone = gp('deadzone').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)

        self.get_logger().info("Joy→Twist (mecanum) ready: joy → cmd_vel")

    def _axis(self, axes, idx):
        return axes[idx] if 0 <= idx < len(axes) else 0.0

    def _dz(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def joy_cb(self, msg: Joy):
        # Dead-man: require enable_button to be held
        enabled = True
        if self.enable_btn >= 0:
            enabled = (0 <= self.enable_btn < len(msg.buttons)) and (msg.buttons[self.enable_btn] == 1)

        if not enabled:
            self.pub.publish(Twist())  # send zero once per Joy message
            return

        # Optional turbo
        turbo = 1.0
        if self.enable_turbo_btn >= 0:
            if 0 <= self.enable_turbo_btn < len(msg.buttons) and msg.buttons[self.enable_turbo_btn] == 1:
                turbo = self.turbo_scale

        # Read axes with deadzone
        lin_x = self._dz(self._axis(msg.axes, self.ax_lin_x))
        lin_y = self._dz(self._axis(msg.axes, self.ax_lin_y))
        # Add D-pad axes to joystick axes
        if self.ax_lin_dpad_x >= 0:
            lin_x += self._dz(self._axis(msg.axes, self.ax_lin_dpad_x))
        if self.ax_lin_dpad_y >= 0:
            lin_y += self._dz(self._axis(msg.axes, self.ax_lin_dpad_y))
        yaw = self._dz(self._axis(msg.axes, self.ax_yaw))

        # Build Twist (x forward, y strafe, z yaw)
        cmd = Twist()
        cmd.linear.x = lin_x * self.s_lin_x * turbo
        cmd.linear.y = lin_y * self.s_lin_y * turbo
        cmd.angular.z = yaw * self.s_yaw * turbo

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
