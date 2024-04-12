#!/usr/bin/python3
import sys
import os
sys.path.append('/home/athimeta/CARVER-Savvy/')

from testlib.common.t import test2
from testlib.c1.tc1 import testc1

testc1()
test2()

import numpy as np
import rclpy
from rclpy.node import Node

# Test import DL libraries
import torch as th
import torch.nn as nn
import torch.nn.functional as F

import gymnasium as gym

import stable_baselines3 as sb3

# Import custom libraries

from AwbuDRL.common.utilities import *

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
        check_gpu()

    def timer_callback(self):
        # self.get_logger().info('Timer callback triggered')
        check_gpu()


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()