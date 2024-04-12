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

# Import custom libraries
import sys
sys.path.append('/home/athimeta/CARVER-Savvy/carverabwu/src/awbu_drl/')

from awbudel_lib.common.t import test
from awbudel_lib.common.utilities import *

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
        test()


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()