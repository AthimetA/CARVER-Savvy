#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import tf_transformations
from nav_msgs.msg import Odometry
from zhbbot_interfaces.srv import RobotSentgoal, Goalreach

class TestNode(Node):
    def __init__(self):
        # Initialize the node with the name 'TestNodeA'
        super().__init__('HestNodeA')


        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Action server to get the goal pose
        self.robot_sent_goal_service_client = self.create_client(RobotSentgoal, 'zhbbot/robot_sent_goal')

        print('HestNodeA Started')

    def robot_sent_goal_service_call(self, goal_x, goal_y):
        # request = RobotSentgoal.Request()
        # request.goal_x = goal_x
        # request.goal_y = goal_y
        # future = self.robot_sent_goal_service_client.call_async(request)

        while not self.robot_sent_goal_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sent Goal service not available, waiting again...')
        request = RobotSentgoal.Request()
        request.goal_x = goal_x
        request.goal_y = goal_y
        future = self.robot_sent_goal_service_client.call_async(request)
        self.get_logger().info('VFF Avoidance: Sent goal to FK node')
        self.get_logger().info(f'future: {future}')
        # while rclpy.ok():
        #     rclpy.spin_once(self)
        #     print('Waiting for response')
        #     if future.done():
        #         try:
        #             response = future.result()
        #             self.get_logger().info(f'Goal reached: {response.status}')
        #             break
        #         except Exception as e:
        #             self.get_logger().info(f'Error: {e}')
        #             break

    def timer_callback(self):
        # diff_x = self.odom_cal_buffer.pose.pose.position.x - self.odom_true_buffer.pose.pose.position.x
        # diff_y = self.odom_cal_buffer.pose.pose.position.y - self.odom_true_buffer.pose.pose.position.y
        # diff_theta = self.odom_cal_buffer.pose.pose.orientation.z - self.odom_true_buffer.pose.pose.orientation.z
        # self.get_logger().info(f'True: {self.odom_true_buffer.pose.pose.position.x}, {self.odom_true_buffer.pose.pose.position.y}, {self.odom_true_buffer.pose.pose.orientation.z}')
        # self.get_logger().info(f'Calculated: {self.odom_cal_buffer.pose.pose.position.x}, {self.odom_cal_buffer.pose.pose.position.y}, {self.odom_cal_buffer.pose.pose.orientation.z}')
        # self.get_logger().info(f'Difference: {diff_x}, {diff_y}, {diff_theta}')
        # self.get_logger().info('-'*20)
        pass

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    test_node.robot_sent_goal_service_call(1.0, 1.0)
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()