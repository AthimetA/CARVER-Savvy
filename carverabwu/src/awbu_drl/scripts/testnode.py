#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node

# Test import DL libraries
import torch as th
import torch.nn as nn
import torch.nn.functional as F

import gymnasium as gym

import stable_baselines3 as sb3

from common.utilities import check_gpu
from common.testcon import testprint

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from awbu_interfaces.srv import RingGoal

class TestNode(Node):
    def __init__(self):
        # Initialize the node with the name 'TestNodeA'
        super().__init__('TestNodeA')
        self.timer_period = 0.1
        self.vx = 0.5
        self.wz = 0.2
        self.count_down_timer = 50 # 5 seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Odom subscriber
        self.odom_topic = '/abwubot/odom'
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Odom ground truth
        self.x_gt = 0.0
        self.y_gt = 0.0
        self.theta_gt = 0.0
        self.odom_gt_topic = '/odom_groud_truth_pose'
        self.odom_gt_sub = self.create_subscription(Odometry, self.odom_gt_topic, self.odom_gt_callback, 10)

        self.get_logger().info('Test node initialized')
        self.get_logger().info('Torch GPU available: {}'.format(th.cuda.get_device_name()))

        # velocity publisher
        self.vel_topic = '/abwubot/cmd_vel'
        # self.vel_topic = '/diff_cont/cmd_vel_unstamped'
        self.vel_pub = self.create_publisher(Twist, self.vel_topic, 10)

        self.task_succeed_client = self.create_client(RingGoal, 'task_succeed')
        self.task_fail_client = self.create_client(RingGoal, 'task_fail')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z

    def odom_gt_callback(self, msg: Odometry):
        self.x_gt = msg.pose.pose.position.x
        self.y_gt = msg.pose.pose.position.y
        self.theta_gt = msg.pose.pose.orientation.z

    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def timer_callback(self):
        self.get_logger().info('='*20)
        self.get_logger().info(f'Diff pose: x: {np.abs(self.x - self.x_gt)}, y: {np.abs(self.y - self.y_gt)}, theta: {np.abs(self.theta - self.theta_gt)}')
        self.get_logger().info(f'Odom pose: x: {self.x}, y: {self.y}, theta: {self.theta} | GT pose: x: {self.x_gt}, y: {self.y_gt}, theta: {self.theta_gt}')
        self.get_logger().info(f'Publishing velocity command to {self.vel_topic} with linear: {self.vx}, angular: {self.wz}')
        self.move(self.vx, self.wz)
        self.count_down_timer -= 1
        if self.count_down_timer == 0:
            self.vx *= -1
            self.wz *= -1
            self.count_down_timer = 50
            req = RingGoal.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('fail service not available, waiting again...')
            self.task_fail_client.call_async(req)




def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()