#!/usr/bin/python3

import numpy as np
import time
import rclpy
import copy
from rclpy.node import Node

# Test import DL libraries
import torch as th
import torch.nn as nn
import torch.nn.functional as F

import gymnasium as gym

import stable_baselines3 as sb3

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from gazebo_msgs.srv import GetEntityState, GetModelList

from settings.constparams import LIDAR_DISTANCE_CAP, NUM_SCAN_SAMPLES

from env_utils import get_simulation_speed, read_stage

from sensor_msgs.msg import LaserScan
from awbu_interfaces.msg import Obstacle

SIM_SPD = get_simulation_speed(read_stage())
RADIUS = 0.5
ALPHA = 0.5

class TestNode(Node):
    def __init__(self):
        # Initialize the node with the name 'TestNodeA'
        super().__init__('TestNodeA')

        # Create a subscriber to the scan topic
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.scan_ranges = np.zeros(NUM_SCAN_SAMPLES)
        self.obstacle_distance_nearest = LIDAR_DISTANCE_CAP

    def scan_callback(self, msg: LaserScan):
        if len(msg.ranges) != NUM_SCAN_SAMPLES:
            print(f"more or less scans than expected! check model.sdf, got: {len(msg.ranges)}, expected: {NUM_SCAN_SAMPLES}")
        # normalize laser values
        self.obstacle_distance_nearest = 1
        for i in range(NUM_SCAN_SAMPLES):
                # Normalize the scan values
                self.scan_ranges[i] = np.clip(float(msg.ranges[i]) / LIDAR_DISTANCE_CAP, 0, 1)
                # Check for obstacles
                if self.scan_ranges[i] < self.obstacle_distance_nearest:
                    self.obstacle_distance_nearest = self.scan_ranges[i]
        # Scale the obstacle distance
        self.obstacle_distance_nearest *= LIDAR_DISTANCE_CAP

        self.get_logger().info(f'Nearest obstacle distance: {self.obstacle_distance_nearest}')
        self.get_logger().info(f'Normalized scan ranges: \n{self.scan_ranges}')
        
def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()