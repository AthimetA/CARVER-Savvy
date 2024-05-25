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
        super().__init__('TestNodeA')
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


        self.odom_cal_sub = self.create_subscription(Odometry, '/odometry/local', self.odom_cal_callback, 10)
        self.odom_cal_buffer = Odometry()
        self.odom_true_sub = self.create_subscription(Odometry, '/odom_groud_truth_pose', self.odom_true_callback, 10)
        self.odom_true_buffer = Odometry()

        # Action server to get the goal pose
        self.robot_sent_goal_service_server = self.create_service(RobotSentgoal, 'zhbbot/robot_sent_goal', self.robot_sent_goal_service_callback)
        self.goal_x = 0.0
        self.goal_y = 0.0

    def robot_sent_goal_service_callback(self, request, response):
        self.goal_x = request.goal_x
        self.goal_y = request.goal_y
        response.status = 'Goal received'
        self.get_logger().info(f'Goal received: {self.goal_x}, {self.goal_y}')
        return response

    # Callback function for the subscriber
    def odom_cal_callback(self, msg:Odometry):
        self.odom_cal_buffer = msg

    def odom_true_callback(self, msg:Odometry):
        self.odom_true_buffer = msg

    def timer_callback(self):
        diff_x = self.odom_cal_buffer.pose.pose.position.x - self.odom_true_buffer.pose.pose.position.x
        diff_y = self.odom_cal_buffer.pose.pose.position.y - self.odom_true_buffer.pose.pose.position.y
        diff_theta = self.odom_cal_buffer.pose.pose.orientation.z - self.odom_true_buffer.pose.pose.orientation.z
        self.get_logger().info(f'True: {self.odom_true_buffer.pose.pose.position.x}, {self.odom_true_buffer.pose.pose.position.y}, {self.odom_true_buffer.pose.pose.orientation.z}')
        self.get_logger().info(f'Calculated: {self.odom_cal_buffer.pose.pose.position.x}, {self.odom_cal_buffer.pose.pose.position.y}, {self.odom_cal_buffer.pose.pose.orientation.z}')
        self.get_logger().info(f'Difference: {diff_x}, {diff_y}, {diff_theta}')
        self.get_logger().info('-'*20)

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()