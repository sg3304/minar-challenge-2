import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class ClickToNavGoal(Node):
    def __init__(self):
        super().__init__('click_to_nav_goal')
        self.sub = self.create_subscription(PointStamped, '/clicked_point', self.send_goal, 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, msg):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = msg.point
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending NavigateToPose goal: x={msg.point.x}, y={msg.point.y}")
        self.client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClickToNavGoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
