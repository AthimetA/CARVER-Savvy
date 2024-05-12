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

from settings.constparams import LIDAR_DISTANCE_CAP

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

        # Cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Odometry subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer to publish cmd_vel
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        # Print the received message
        self.get_logger().info(f'Received: {msg}')

    def timer_callback(self):
        # Create a new Twist message
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5

        # Publish the message
        self.cmd_vel_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()