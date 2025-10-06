# #!/usr/bin/env python3
# """
# ROS2 Gym Env for TurtleBot3 + Nav2 simulation

# * Observation: downsampled LIDAR scan (float32 vector)
# * Action: [linear_x, angular_z]
# * Reset: calls Gazebo reset_world
#   """

# import time
# import numpy as np
# import gym
# from gym import spaces

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# from std_srvs.srv import Empty

# class RosLidarNavEnv(gym.Env):
#     metadata = {'render.modes': ['human']}

#     def __init__(self,
#                 scan_topic='/scan',
#                 cmd_vel_topic='/cmd_vel',
#                 reset_service='/gazebo/reset_world',
#                 num_scan_points=72,
#                 linear_lim=(0.0, 0.22),
#                 angular_lim=(-1.5, 1.5),
#                 max_steps=300):
#         super().__init__()

#         self.node = rclpy.create_node('rl_lidar_env')
#         self.last_scan = None

#         self.node.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
#         self.cmd_pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
#         self.reset_client = self.node.create_client(Empty, reset_service)

#         # obs: downsampled scan
#         self.num_scan_points = num_scan_points
#         self.observation_space = spaces.Box(low=0.0, high=1.0,
#                                             shape=(num_scan_points,),
#                                             dtype=np.float32)

#         # action: linear & angular velocity
#         low = np.array([linear_lim[0], angular_lim[0]], dtype=np.float32)
#         high = np.array([linear_lim[1], angular_lim[1]], dtype=np.float32)
#         self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)

#         self.max_steps = max_steps
#         self.step_count = 0
#         self.collision_thresh = 0.12

#     def _scan_cb(self, msg):
#         # preprocess scan into fixed-length vector
#         ranges = np.array(msg.ranges)
#         ranges = np.nan_to_num(ranges, nan=msg.range_max)
#         ranges = np.clip(ranges, 0.0, msg.range_max)
#         # downsample evenly
#         idx = np.linspace(0, len(ranges)-1, self.num_scan_points).astype(int)
#         scan_ds = ranges[idx] / msg.range_max  # normalize 0–1
#         self.last_scan = scan_ds.astype(np.float32)

#     def reset(self):
#         # call Gazebo reset
#         if not self.reset_client.service_is_ready():
#             self.reset_client.wait_for_service(timeout_sec=5.0)
#         req = Empty.Request()
#         fut = self.reset_client.call_async(req)
#         rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)

#         self.step_count = 0
#         # wait for first scan
#         t0 = time.time()
#         while self.last_scan is None and time.time()-t0 < 5:
#             rclpy.spin_once(self.node, timeout_sec=0.1)
#         return self.last_scan.copy()

#     def step(self, action):
#         self.step_count += 1
#         twist = Twist()
#         twist.linear.x = float(action[0])
#         twist.angular.z = float(action[1])
#         self.cmd_pub.publish(twist)

#         # wait a bit for scan update
#         rclpy.spin_once(self.node, timeout_sec=0.1)
#         obs = self.last_scan.copy()

#         # reward shaping
#         reward = action[0] * 1.0  # forward motion reward
#         if np.min(obs) < self.collision_thresh:
#             reward -= 5.0
#             done = True
#             info = {'reason': 'collision'}
#         elif self.step_count >= self.max_steps:
#             done = True
#             info = {'reason': 'timeout'}
#         else:
#             done = False
#             info = {}

#         return obs, reward, done, info

#     def close(self):
#         self.node.destroy_node()
# def main(args=None):
#     rclpy.init(args=args)
#     odometry_node = RosLidarNavEnv()
#     rclpy.spin(odometry_node)
#     odometry_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
