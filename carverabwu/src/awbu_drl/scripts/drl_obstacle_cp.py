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

from geometry_msgs.msg import Twist , Pose
from nav_msgs.msg import Odometry

from gazebo_msgs.srv import GetEntityState, GetModelList

from settings.constparams import LIDAR_DISTANCE_CAP

from env_utils import get_simulation_speed, read_stage

from sensor_msgs.msg import LaserScan
from awbu_interfaces.msg import Obstacle

SIM_SPD = get_simulation_speed(read_stage())
RADIUS = 0.5
ALPHA = 0.5

class ObstacleCP(Node):
    def __init__(self):
        super().__init__('ObstacleCP')
        
        self.get_entity_state_client    = self.create_client(GetEntityState, '/gazebo_drl/get_entity_state')
        self.get_model_list_client      = self.create_client(GetModelList, '/get_model_list')

        # Gazebo model list subscriber
        self.model_list = self.get_model_list()
        self.obstacle_list =  [model for model in self.model_list if 'obstacle' in model]

        self.get_logger().info(f'Obstacle List: {self.obstacle_list}')

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

            if time_diff >= self.control_loop_period:
                self.start_loop_time = self.time_sec

                # print("==========")
                ID_LIST = [] 
                CENTER_X = []
                CENTER_Y = []
                VELOCITY_X = []
                VELOCITY_Y = []
                CP_LIST = []

                # Get the current state of the obstacles
                for obstacle in self.obstacle_list:

                    # Get the initial pose and twist of the obstacle
                    pose, twist = self.get_entity_state(obstacle)
                    
                    robot_pos_x = self.position.x
                    robot_pos_y = self.position.y
                    robot_vel_x = self.linear_twist.x
                    robot_vel_y = self.linear_twist.y

                    obs_pos_x = pose.position.x
                    obs_pos_y = pose.position.y
                    obs_vel_x = twist.linear.x
                    obs_vel_y = twist.linear.y

                    # self.get_logger().info(f'Name: {obstacle}, Pose: {pose.position.x}, {pose.position.y}, \
                    #                        Twist: {twist.linear.x}, {twist.linear.y}, {twist.linear.z}')
                    
                    obs_pose = np.array([obs_pos_x , obs_pos_y])
                    robot_post = np.array([robot_pos_x , robot_pos_y])

                    dist = np.linalg.norm(robot_post - obs_pose)

                    if dist < LIDAR_DISTANCE_CAP + 0.5:

                        Dist_o = dist - RADIUS
                        Vr = np.array([robot_vel_x , robot_vel_y])
                        Vo = np.array([obs_vel_x , obs_vel_y])
                        Vr_prime = Vr - Vo
                        t = Dist_o / np.sqrt(Vr_prime[0]**2 + Vr_prime[1]**2)

                        Pc_ttc = min([ 1, self.control_loop_period / t])
                        Imax = self.max_scan
                        Imin = self.min_scan 

                        Pc_dto = (Imax - Dist_o) / (Imax - Imin)

                        
                        CP = ALPHA * Pc_ttc + (1-ALPHA) * Pc_dto
                        # print("Pc_ttc : " ,Pc_ttc)
                        # print("Pc_dto : " ,Pc_dto)
                        # print("Collision Probability (CP) : " , CP)

                        ID_LIST.append(int(obstacle[-1]))
                        CENTER_X.append(pose.position.x)
                        CENTER_Y.append(pose.position.y)
                        VELOCITY_X.append(twist.linear.x)
                        VELOCITY_Y.append(twist.linear.y)
                        CP_LIST.append(CP)

                self._obstacle_pubish(ID_LIST , CENTER_X , CENTER_Y , VELOCITY_X , VELOCITY_Y , CP_LIST)

                
    def _obstacle_pubish(self,_id,center_x,center_y,velocity_x,velocity_y,CP):

        self._OBSTACLE = Obstacle()

        self._OBSTACLE.id = _id
        self._OBSTACLE.pose_x = center_x
        self._OBSTACLE.pose_y = center_y 
        self._OBSTACLE.velocity_x = velocity_x
        self._OBSTACLE.velocity_y = velocity_y
        self._OBSTACLE.cp = CP

        self.CP_publisher.publish(self._OBSTACLE)

    def scan_callback(self ,msg: LaserScan):
        self.scan = msg
        self.max_scan = msg.range_max
        self.min_scan = msg.range_min

    
    def get_odometry(self, odom: Odometry):
        self.position = odom.pose.pose.position
        self.orientation = odom.pose.pose.orientation
        self.linear_twist = odom.twist.twist.linear
        self.angular_twist = odom.twist.twist.angular


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
    test_node = ObstacleCP()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()