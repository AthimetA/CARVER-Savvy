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

from gazebo_msgs.srv import GetEntityState, GetModelList , GetModelState
from drl_obstacle_control import DynamicObstacle

from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock

from settings.constparams import TOPIC_CLOCK , LIDAR_DISTANCE_CAP

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

        self.subscription = self.create_subscription(
        LaserScan,
        '/scan',
        self.scan_callback,
        10)

        self.CP_publisher = self.create_publisher(Obstacle, '/abwubot/obstacleCP', 10)
        self.odom_topic_name = '/abwubot/odom'
        self.sub_odom = self.create_subscription(
            Odometry,
            self.odom_topic_name,
            self.get_odometry,
            10)

        self.scan = None
        self.position = None


        time.sleep(3)
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
                        ID_LIST = [] 
                        CENTER_X = []
                        CENTER_Y = []
                        VELOCITY_X = []
                        VELOCITY_Y = []
                        CP_LIST = []

                        # Get the initial pose and twist of the obstacle
                        pose, twist = self.get_entity_state(obstacle)
                        # self.get_logger().info(f'Name: {obstacle}, Pose: {pose.position.x}, {pose.position.y}, \
                        #                        Twist: {twist.linear.x}, {twist.linear.y}, {twist.linear.z}')
                        
                        ob_pose = np.array([pose.position.x , pose.position.y])
                        robot_post = np.array([self.position.x , self.position.y])

                        dist = np.linalg.norm(robot_post - ob_pose)

                        if dist < LIDAR_DISTANCE_CAP :

                            
                            Dist_o = dist - RADIUS
                            Vr = np.array([self.linear_twist.x , self.linear_twist.y])
                            Vo = np.array([twist.linear.x , twist.linear.y])
                            Vr_prime = Vr - Vo
                            t = Dist_o / np.sqrt(Vr_prime[0]**2 + Vr_prime[1]**2)

                            Pc_ttc = min([ 1, self.control_loop_period / t])
                            Imax = self.max_scan
                            Imin = self.min_scan 

                            Pc_dto = (Imax - Dist_o) / (Imax - Imin)

                            
                            CP = ALPHA * Pc_ttc + (1-ALPHA) * Pc_dto
                            print("==========")
                            print("Pc_ttc : " ,Pc_ttc)
                            print("Pc_dto : " ,Pc_dto)
                            print("Collision Probability (CP) : " , CP)

                            ID_LIST.append(0.0)
                            CENTER_X.append(pose.position.x)
                            CENTER_Y.append(pose.position.y)
                            VELOCITY_X.append(twist.linear.x)
                            VELOCITY_Y.append(twist.linear.y)
                            CP_LIST.append(CP)

                    
    
    def _obstacle_pubish(self,_id,center_x,center_y,velocity_x,velocity_y,CP):

        self._OBSTACLE = Obstacle()

        self._OBSTACLE.id = _id
        self._OBSTACLE.pose_x = center_x
        self._OBSTACLE.pose_y = center_y 
        self._OBSTACLE.velocity_x = velocity_x
        self._OBSTACLE.velocity_y = velocity_y
        self._OBSTACLE.cp = CP

        self.CP_publisher.publish(self._OBSTACLE)

    def scan_callback(self ,msg):
        self.scan = msg
        self.max_scan = msg.range_max
        self.min_scan = msg.range_min

    
    def get_odometry(self, odom):
        self.position = odom.pose.pose.position
        self.orientation = odom.pose.pose.orientation
        self.linear_twist = odom.twist.twist.linear
        self.angular_twist = odom.twist.twist.angular


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