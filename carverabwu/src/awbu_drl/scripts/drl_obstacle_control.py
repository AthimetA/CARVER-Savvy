#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert, Tomas

# original implementation from: 
# https://github.com/tomasvr/turtlebot3_drlnav
# https://github.com/ailabspace/drl-based-mapless-crowd-navigation-with-perceived-risk
# 

# Modified by:  Athimet Aiewcharoen     , FIBO, KMUTT
#               Tanut   Bumrungvongsiri , FIBO, KMUTT
# Date : 2024-05-26

import time
import numpy as np
import rclpy
from rclpy.node import Node
import yaml

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import GetModelList, SetEntityState, GetEntityState

from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock

from settings.constparams import EPISODE_TIMEOUT_SECONDS

# Topic name imports
from settings.constparams import TOPIC_CLOCK

from awbu_interfaces.srv import ObstacleStart

from env_utils import get_simulation_speed, read_stage, bcolors

OBSTACLE_VELOCITY_SCALING = 1.0

from ament_index_python import get_package_share_directory
class ObstacleHandler(Node):
    def __init__(self):
        super().__init__('ObstacleHandler')
        self.stage = read_stage()
        self.get_logger().info(bcolors.OKGREEN + f'Stage: {self.stage}' + bcolors.ENDC)
        self.sim_speed = get_simulation_speed(stage=self.stage)
        
        # Initialise services servers
        self.obstacle_start_srv = self.create_service(ObstacleStart, '/obstacle_start', self.obstacle_start_callback)
        self.obstacle_status = False

        # Gazebo service client
        self.set_entity_state_client    = self.create_client(SetEntityState, '/gazebo_drl/set_entity_state')
        self.get_entity_state_client    = self.create_client(GetEntityState, '/gazebo_drl/get_entity_state')
        self.get_model_list_client      = self.create_client(GetModelList, '/get_model_list')
        self.reset_simulation_client    = self.create_client(Empty, '/reset_world')

        self.reset_simulation()

        # Gazebo model list subscriber
        self.model_list = self.get_model_list()
        self.obstacle_list =  self.init_obstacles()
        self.get_logger().info(f'Obstacle list: {self.obstacle_list}')

        # Creace a publisher for the obstacle control
        self.obstacle_control_pub_list = [self.create_publisher(Twist, f'/{obstacle}/cmd_vel', 10) for obstacle in self.obstacle_list]

        # Control loop
        self.new_velo_time = 0.1 # seconds # Time to update the velocity
        self.control_loop_period = self.new_velo_time / self.sim_speed
        self.twist_list = [Twist() for _ in range(len(self.obstacle_list))]

        self.timer = self.create_timer(self.control_loop_period, self.timer_callback)

    def timer_callback(self):
        if self.stage == 1:
            self.stage_1_obstacle_control()
        elif self.stage == 2:
            self.stage_2_obstacle_control()
        elif self.stage == 4:
            self.stage_4_obstacle_control()

    def stage_1_obstacle_control(self):
        if self.obstacle_status:
            # Generate new velocity
            for obs_idx, obstacle in enumerate(self.obstacle_list):
                self.twist_list[obs_idx].linear.x = np.random.uniform(0, 1) * OBSTACLE_VELOCITY_SCALING
                self.twist_list[obs_idx].angular.z = np.random.uniform(0, 1) * OBSTACLE_VELOCITY_SCALING
                self.obstacle_control_pub_list[obs_idx].publish(self.twist_list[obs_idx])

    def stage_2_obstacle_control(self):
        if self.obstacle_status:
            # Half of the obstacles move in the +x direction and the other half in the -x direction
            for obs_idx, obstacle in enumerate(self.obstacle_list):
                # self.twist_list[obs_idx].linear.x = np.random.uniform(-0.1, 1.0) * OBSTACLE_VELOCITY_SCALING
                self.twist_list[obs_idx].linear.x = np.random.uniform(0.4, 1.0) * OBSTACLE_VELOCITY_SCALING
                # self.twist_list[obs_idx].angular.z = np.random.uniform(-0.01, 0.01) * OBSTACLE_VELOCITY_SCALING
                self.obstacle_control_pub_list[obs_idx].publish(self.twist_list[obs_idx])

    def stage_4_obstacle_control(self):
        if self.obstacle_status:

            for obstacle in self.obstacle_list:
                if obstacle.initial_pose.position.y > 0:
                    init_y = np.random.uniform(2.0, 8.0)
                    target_y = np.random.uniform(-8.0, -2.0)
                else:
                    init_y = np.random.uniform(-8.0, -2.0)
                    target_y = np.random.uniform(2.0, 8.0)

                # Set the initial pose of the obstacles
                init_Pose = Pose()
                init_Pose.position.x = obstacle.initial_pose.position.x
                init_Pose.position.y = init_y
                init_Pose.position.z = obstacle.initial_pose.position.z
                init_Pose.orientation = obstacle.initial_pose.orientation
                obstacle.set_initial_pose(init_Pose)

                # Set the final pose of the obstacles
                target_Pose = Pose()
                target_Pose.position.x = obstacle.initial_pose.position.x
                target_Pose.position.y = target_y
                target_Pose.position.z = obstacle.initial_pose.position.z
                target_Pose.orientation = obstacle.initial_pose.orientation
                obstacle.set_target_pose(target_Pose)

                # Calculate the velocity to reach the target pose
                obstacle.linear_path_velocity_calculation()

                # Set entity state
                self.set_entity_state(obstacle.name, obstacle.initial_pose, obstacle.target_twist)

                self.obstacle_status = False


    def obstacle_start_callback(self, request: ObstacleStart.Request, response: ObstacleStart.Response):
        response.obstacle_status = True
        self.obstacle_status = True
        return response

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

    def reset_simulation(self):
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')
        self.reset_simulation_client.call_async(Empty.Request())

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

    def set_entity_state(self, entity_name: str, pose: Pose, twist: Twist):
        request = SetEntityState.Request()
        request.state.name = entity_name
        request.state.pose = pose
        request.state.twist = twist
        request.state.reference_frame = 'world'
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            # self.get_logger().info(f'Setting entity state for {entity_name}...')
            future = self.set_entity_state_client.call_async(request)
            # rclpy.spin_until_future_complete(self, future)
            response = future.result()
            # self.get_logger().info(f'Response: {response}')
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def init_obstacles(self):
        # Initialize the obstacles
        obstacle_list = []
        for obstacle in self.model_list:
            if 'obstacle' in obstacle:
                # Get the initial pose and twist of the obstacle
                pose, twist = self.get_entity_state(obstacle)
                obstacle_list.append(DynamicObstacle(name=obstacle, initial_pose=pose))
        return obstacle_list

class DynamicObstacle:
    def __init__(self,
    name: str,
    initial_pose: Pose,
    ):
        # Obstacle information
        self.name = name
        self.initial_pose = initial_pose

        self.target_twist = Twist()

        self.time_out = EPISODE_TIMEOUT_SECONDS

    def set_initial_pose(self, pose: Pose):
        self.initial_pose = pose

    def set_target_pose(self, pose: Pose):
        self.target_pose = pose
        self.linear_path_velocity_calculation()

    def linear_path_velocity_calculation(self):
        # Calculate the target twist based on the target pose
        distance_x = self.target_pose.position.x - self.initial_pose.position.x
        distance_y = self.target_pose.position.y - self.initial_pose.position.y

        # Calculate the Twist to reach the target pose
        self.target_twist = Twist()
        self.target_twist.linear.x = distance_x / self.time_out
        self.target_twist.linear.y = distance_y / self.time_out

    def __repr__(self) -> str:
        return f'{self.name}'
        

def main(args=None):
    rclpy.init(args=args)
    test_node = ObstacleHandler()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()