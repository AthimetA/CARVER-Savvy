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

class TestNode(Node):
    def __init__(self):
        # Initialize the node with the name 'TestNodeA'
        super().__init__('TestNodeA')
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Test node initialized')
        # # Check if GPU is available
        # if th.cuda.is_available():
        #     self.get_logger().info('Torch GPU available: {}'.format(th.cuda.get_device_name()))
        # else:
        #     self.get_logger().info('Torch GPU not available')
        self.get_logger().info('Torch GPU available: {}'.format(th.cuda.get_device_name()))
        self.get_logger().info('====================')
        self.get_logger().info(f'Test: {check_gpu()}')

        # velocity publisher
        self.vel_pub = self.create_publisher(Twist, '/abwubot/cmd_vel', 10)
    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def timer_callback(self):
        self.move(0.5, 0.2)



def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()