#!/usr/bin/python3

import numpy as np
import time
import rclpy
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
from drl_obstacle_control import DynamicObstacle

from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock

from settings.constparams import TOPIC_CLOCK

from env_utils import get_simulation_speed, read_stage

SIM_SPD = get_simulation_speed(read_stage())

class TestNode(Node):
    def __init__(self):
        # Initialize the node with the name 'TestNodeA'
        super().__init__('TestNodeA')
        
        self.get_entity_state_client    = self.create_client(GetEntityState, '/gazebo_drl/get_entity_state')
        self.get_model_list_client      = self.create_client(GetModelList, '/get_model_list')
        # Gazebo model list subscriber
        self.model_list = self.get_model_list()
        self.obstacle_list =  self.init_obstacles()

        self.get_logger().info(f'Model List: {self.model_list}')
        self.get_logger().info(f'Obstacle List: {self.obstacle_list}')
        self.get_logger().info('TestNodeA has been initialized')


        # Clock subscriber
        self.control_loop_hz = 30 * SIM_SPD
        self.control_loop_period = 1.0 / self.control_loop_hz
        self.time_sec = time.perf_counter()
        self.start_loop_time = self.time_sec


        self.control_loop()

    def control_loop(self):

        # Control loop
        while True:
            
            self.time_sec = time.perf_counter()

            time_diff = self.time_sec - self.start_loop_time

            if self.time_sec - self.start_loop_time >= self.control_loop_period:
                self.start_loop_time = self.time_sec
                # Get the current state of the obstacles
                for obstacle in self.model_list:
                    if 'obstacle' in obstacle:
                        # Get the initial pose and twist of the obstacle
                        pose, twist = self.get_entity_state(obstacle)
                        self.get_logger().info(f'Name: {obstacle}, Pose: {pose.position.x}, {pose.position.y}, Twist: {twist.linear.x}, {twist.linear.y}, {twist.linear.z}')
                    

        


    def init_obstacles(self):
        # Initialize the obstacles
        obstacle_list = []
        for obstacle in self.model_list:
            if 'obstacle' in obstacle:
                # Get the initial pose and twist of the obstacle
                pose, twist = self.get_entity_state(obstacle)
                self.get_logger().info(f'Name: {obstacle}, Pose: {pose}, Twist: {twist}')
                obstacle_list.append(DynamicObstacle(name=obstacle, initial_pose=pose))
        return obstacle_list

    def get_model_list(self):
        request = GetModelList.Request()
        while not self.get_model_list_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            future = self.get_model_list_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            return response.model_names

        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def get_entity_state(self, entity_name):
        request = GetEntityState.Request()
        request.name = entity_name
        request.reference_frame = 'world'
        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            future = self.get_entity_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            return response.state.pose, response.state.twist
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()