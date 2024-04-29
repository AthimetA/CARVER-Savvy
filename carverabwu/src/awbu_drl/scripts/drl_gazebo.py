#!/usr/bin/env python3

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

# Modified by: Athimet Aiewcharoen, FIBO, KMUTT
# Date: 2024-04-24

import os
import random
import math
import copy
import numpy as np
import time

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

import xml.etree.ElementTree as ET
from settings.constparams import ENABLE_TRUE_RANDOM_GOALS, ARENA_LENGTH, ARENA_WIDTH, ENABLE_DYNAMIC_GOALS

# GENERAL SETTINGS
from settings.constparams import ENABLE_BACKWARD, ENABLE_DYNAMIC_GOALS
# ENVIRONMENT SETTINGS 
# Sensor
from settings.constparams import TOPIC_SCAN, TOPIC_VELO, TOPIC_ODOM, TOPIC_GOAL,\
                                 LIDAR_DISTANCE_CAP, THRESHOLD_COLLISION, THREHSOLD_GOAL,\
                                 ENABLE_MOTOR_NOISE
# Simulation Environment Settings
# Arena dimensions
from settings.constparams import ARENA_LENGTH, ARENA_WIDTH
# Obstacle settings
from settings.constparams import MAX_NUMBER_OBSTACLES, OBSTACLE_RADIUS 
# General
from settings.constparams import EPISODE_TIMEOUT_SECONDS, SPEED_LINEAR_MAX, SPEED_ANGULAR_MAX,\
                                 LINEAR_VELOCITY_LOC, ANGULAR_VELOCITY_LOC

# DRL ALGORITHM SETTINGS
from settings.constparams import UNKNOWN, SUCCESS, COLLISION_WALL, COLLISION_OBSTACLE, TIMEOUT, TUMBLE

# Robot specific settings
from settings.constparams import NUM_SCAN_SAMPLES

from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from awbu_interfaces.srv import DrlStep, Goal, EnvReady

import reward as rw
from common import utilities as util


from env_utils import ObstacleManager, Robot

MAX_GOAL_DISTANCE = math.sqrt(ARENA_LENGTH**2 + ARENA_WIDTH**2)
REST_SIMULATION_PAUSE = 0.5  # seconds

class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        '''
        
        Goal Box entity
        
        '''

        self._entity_dir_path = os.environ['SIM_MODEL_PATH'] + '/goal_box'
        # self.get_logger().info("Goal entity directory path: " + self._entity_dir_path)
        self._entity_path = os.path.join(self._entity_dir_path, 'model.sdf')
        self.entity = open(self._entity_path, 'r').read()
        self.entity_name = 'goal_box'

        '''
        
        Initialize variables
        
        '''

        # --------------- Constants --------------- #
        # Topics
        self.scan_topic = TOPIC_SCAN
        self.velo_topic = TOPIC_VELO
        self.odom_topic = TOPIC_ODOM
        self.clock_topic = '/clock'
        self.obstacle_odom_topic = '/obstacle/odom'
        
        # --------------- Robot --------------- #
        self.robot = Robot()
        # --------------- Goal --------------- #
        self.goal_ready = False
        self.goal_x, self.goal_y = 0.0, 0.0

        # --------------- Laser Scanner --------------- #
        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance_nearest = LIDAR_DISTANCE_CAP
        self.obstacle_distances = [np.inf] * MAX_NUMBER_OBSTACLES

        # --------------- Time and Episode --------------- #
        self.episode_timeout = EPISODE_TIMEOUT_SECONDS
        # Episode variables
        self.episode_deadline = np.inf
        self.reset_deadline = False

        self._EP_done = False
        self._EP_succeed = UNKNOWN
        self.local_step = 0 # Local step counter

        # --------------- ROS Parameters --------------- #
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        '''
        
        Obstacle Manager
        
        '''
        self.obstacle_manager = ObstacleManager()
        # Pointers to the obstacle coordinates variable in the ObstacleManager class
        self.obstacle_coordinates = self.obstacle_manager.obstacle_coordinates
        self.get_logger().info(f"Obstacle coordinates: {self.obstacle_coordinates}")


        '''
        
        Initialize Node
        
        '''

        # Initialise publishers
        self.cmd_vel_pub                = self.create_publisher(Twist, self.velo_topic, qos)

        # subscribers
        self.odom_sub                   = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.scan_sub                   = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.clock_sub                  = self.create_subscription(Clock, self.clock_topic, self.clock_callback, qos_profile=qos_clock)
        self.obstacle_odom_sub          = self.create_subscription(Odometry, self.obstacle_odom_topic, self.obstacle_odom_callback, qos)

        # Initialise services clients
        self.delete_entity_client       = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_entity_client        = self.create_client(SpawnEntity, '/spawn_entity')
        self.reset_simulation_client    = self.create_client(Empty, '/reset_world')
        self.gazebo_pause               = self.create_client(Empty, '/pause_physics')
        self.gazebo_unpause             = self.create_client(Empty, '/unpause_physics')

        # Initialise services servers
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)
        self.goal_comm_server = self.create_service(EnvReady, 'env_comm', self.env_comm_callback)

        # Initialize Node
        self.init_drl()
        self.get_logger().info("DRL Gazebo node has been started.")

        # Debug timer
        self.timer = self.create_timer(1.0, self.test_callback)
        self.real_node_time_sec = 0

    def test_callback(self):
        self.real_node_time_sec += 1

    '''
    
    Gazebo simulation control functions
    
    '''
    def reset_simulation(self):
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')
        self.reset_simulation_client.call_async(Empty.Request())

    def delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.entity_name
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.delete_entity_client.call_async(req)

    def pause_simulation(self):
        # Pause simulation
        while not self.gazebo_pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pause gazebo service not available, waiting again...')
        self.gazebo_pause.call_async(Empty.Request())
            
    def unpause_simulation(self):
        # Unpause simulation
        while not self.gazebo_unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('unpause gazebo service not available, waiting again...')
        self.gazebo_unpause.call_async(Empty.Request())

    def spawn_entity(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.entity
        req.initial_pose = goal_pose
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.spawn_entity_client.call_async(req)
        self.get_logger().info("Entity spawned")

    '''
    
    Client Callbacks Functions
    
    '''
    def odom_callback(self, msg: Odometry):
        # Update the robot position
        self.robot.update_position(msg)

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

    def clock_callback(self, msg: Clock):
        # Get current time
        self.time_sec = msg.clock.sec
        # Reset episode deadline
        if not self.reset_deadline:
            return
        # Reset episode deadline
        episode_time = self.episode_timeout
        # Set deadline
        self.episode_deadline = self.time_sec + episode_time
        # Reset variables
        self.reset_deadline = False

    def obstacle_odom_callback(self, msg: Odometry):
        if 'obstacle' in msg.child_frame_id:
            # robot_pos = msg.pose.pose.position
            # obstacle_id = int(msg.child_frame_id[-1]) - 1
            # diff_x = self.robot_x - robot_pos.x
            # diff_y = self.robot_y - robot_pos.y
            # self.obstacle_distances[obstacle_id] = math.sqrt(diff_y**2 + diff_x**2)
            pass # Not used in this version
        else:
            print("ERROR: received odom was not from obstacle!")

    '''
    
    Service functions
    
    '''
    # --------------- Goal Service --------------- #
    #
    # Callback function for the goal service
    #
    # Will be called when agent starts a new episode
    #
    def env_comm_callback(self, request: EnvReady.Request, response: EnvReady.Response):
        response.env_status = self.goal_ready
        return response
    
    def get_state(self,
        action_linear_previous: float,
        action_angular_previous: float
    ):
        state = copy.deepcopy(self.scan_ranges)                                                     # range: [ 0, 1]
        state.append(float(np.clip((self.robot.distance_to_goal / MAX_GOAL_DISTANCE), 0, 1)))       # range: [ 0, 1]
        state.append(float(self.robot.goal_angle) / math.pi)                                        # range: [-1, 1]
        state.append(float(action_linear_previous))                                                 # range: [-1, 1]
        state.append(float(action_angular_previous))                                                # range: [-1, 1]
        self.local_step += 1
        # Check if the episode is done
        self.episode_check()
        return state
    
    def episode_check(self):
        # Success
        if self.robot.distance_to_goal < THREHSOLD_GOAL:
            self._EP_succeed = SUCCESS
        # Collision
        elif self.obstacle_distance_nearest < THRESHOLD_COLLISION:
            self._EP_succeed = COLLISION_OBSTACLE
        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self._EP_succeed = TIMEOUT
        # Tumble
        # elif self.robot.tilt > 0.06 or self.robot.tilt < -0.06:
        #     self._EP_succeed = TUMBLE

        # Check if the episode is done [Success, Collision, Timeout, Tumble] 
        if self._EP_succeed is not UNKNOWN:
            self.episode_done(self._EP_succeed)
    
    def episode_done(self, status: int):
        # Stop the robot 
        self.cmd_vel_pub.publish(Twist())
        # Reset the episode deadline
        self.episode_deadline = np.inf
        self._EP_done = True
        if status == SUCCESS:
            self.get_logger().info("Episode done, Agent reached the goal!")
        elif status == COLLISION_OBSTACLE:
            self.get_logger().info("Episode done, Agent collided with obstacle!")
        elif status == COLLISION_WALL:
            self.get_logger().info("Episode done, Agent collided with wall!")
        elif status == TIMEOUT:
            self.get_logger().info("Episode done, Agent reached the timeout!")
        elif status == TUMBLE:
            self.get_logger().info("Episode done, Agent tumbled!")
        else:
            self.get_logger().info("Episode done, Unknown status!")
    
    def initalize_episode(self, response: DrlStep.Response):
        '''

        DRLStep service [Request/Response] structure :
        
        float32[] action
        float32[] previous_action
        ---
        float32[] state
        float32 reward
        bool done
        uint32 success
        float32 distance_traveled
        
        
        '''
        # Pause the simulation
        self.pause_simulation()

        # Delete the goal entity
        self.delete_entity()

        # Reset the simulation
        self.reset_simulation()
        time.sleep(REST_SIMULATION_PAUSE)

        # Generate a new goal
        self.generate_goal_pose(self.robot.x, self.robot.y, OBSTACLE_RADIUS)
        # Update the robot goal
        self.robot.update_goal(self.goal_x, self.goal_y)
        # Reset the robot
        self.robot.reset()

        # Spawn the goal entity
        self.spawn_entity()
    
        # Reset the episode variables
        self.reset_deadline = True

        response.state = self.get_state(action_linear_previous=0.0, action_angular_previous=0.0)
        response.reward = 0.0
        response.done = False
        response.success = UNKNOWN
        response.distance_traveled = 0.0

        self.get_logger().info(f"New episode started, Goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")

        rw.reward_initalize(self.robot.distance_to_goal)

        # Unpause the simulation
        self.unpause_simulation()

        return response
    
    def step_comm_callback(self, request: DrlStep.Request, response: DrlStep.Response):

        '''

        DRLStep service [Request/Response] structure :
        
        float32[] action
        float32[] previous_action
        ---
        float32[] state
        float32 reward
        bool done
        uint32 success
        float32 distance_traveled
        
        
        '''
        # Check if the episode has been initialized
        if len(request.action) == 0:
            return self.initalize_episode(response)

        if ENABLE_MOTOR_NOISE:
            request.action[LINEAR_VELOCITY_LOC] += np.clip(np.random.normal(0, 0.05), -0.1, 0.1)
            request.action[ANGULAR_VELOCITY_LOC] += np.clip(np.random.normal(0, 0.05), -0.1, 0.1)

        # Un-normalize actions
        if ENABLE_BACKWARD:
            action_linear = request.action[LINEAR_VELOCITY_LOC] * SPEED_LINEAR_MAX
        else:
            action_linear = (request.action[LINEAR_VELOCITY_LOC] + 1) / 2 * SPEED_LINEAR_MAX
        action_angular = request.action[ANGULAR_VELOCITY_LOC] * SPEED_ANGULAR_MAX

        # Publish action cmd
        twist = Twist()
        twist.linear.x = action_linear
        twist.angular.z = action_angular
        # self.get_logger().info(f"Publishing action: linear: {twist.linear.x:.2f} angular: {twist.angular.z:.2f}")
        self.cmd_vel_pub.publish(twist)

        # Prepare repsonse
        response.state = self.get_state(
            action_linear_previous  = request.previous_action[LINEAR_VELOCITY_LOC],
            action_angular_previous = request.previous_action[ANGULAR_VELOCITY_LOC])
        # Calculate reward
        response.reward = rw.get_reward(
            succeed=self._EP_succeed,
            action_linear=action_linear,
            action_angular=action_angular,
            distance_to_goal=self.robot.distance_to_goal,
            goal_angle=self.robot.goal_angle,
            min_obstacle_distance=self.obstacle_distance_nearest,
        )
        # Update the episode status
        response.done = self._EP_done
        response.success = self._EP_succeed
        response.distance_traveled = 0.0
        # Check if the episode is done
        if self._EP_done:
            self.get_logger().info(f"Episode done! Reward: {response.reward:.2f} Success: {self._EP_succeed}")
            response.distance_traveled = self.robot.distance_traveled
            # Reset variables
            self._EP_succeed = UNKNOWN
            self.local_step = 0
            self._EP_done = False
        if self.local_step % 2 == 0:
            print(f"T: {self.time_sec:<8}RT:{self.real_node_time_sec:<8}EPD: {self.episode_deadline:<8}\t", end='')
            print(f"Reward: {response.reward:<8.2f}DTG: {self.robot.distance_to_goal:<8.2f}AG: {math.degrees(self.robot.goal_angle):.1f}°\t", end='')
            print(f"MinOBD: {self.obstacle_distance_nearest:<8.2f}Alin: {request.action[LINEAR_VELOCITY_LOC]:<7.1f}Aturn: {request.action[ANGULAR_VELOCITY_LOC]:<7.1f}")
        return response

    '''

    DRL Genral functions

    '''

    def init_drl(self)->None:
        # Initialize the DRL node
        self.pause_simulation()
        self.delete_entity() # if entity exists delete it
        self.reset_simulation() # Reset the simulation
        time.sleep(REST_SIMULATION_PAUSE)
        self.unpause_simulation() 
        self.get_logger().info(f"DRL Gazebo node has been initialized, Simulation Paused")
        self.get_logger().info(f"Please start the episode by calling the service")
        # Goal is ready
        self.goal_ready = True
    
    '''
    
    Goal generation functions
    
    '''

    def goal_is_valid(self, goal_x: float, goal_y: float)->bool:
        if goal_x > ARENA_LENGTH/2 or goal_x < -ARENA_LENGTH/2 or goal_y > ARENA_WIDTH/2 or goal_y < -ARENA_WIDTH/2:
            return False
        for obstacle in self.obstacle_coordinates:
            # Obstacle is defined by 4 points [top_right, bottom_right, bottom_left, top_left] with [x, y] coordinates
            if goal_x < obstacle[0][0] and goal_x > obstacle[2][0] and goal_y < obstacle[0][1] and goal_y > obstacle[2][1]: # check if goal is inside the obstacle
                    return False
        return True

    def generate_goal_pose(self, robot_x: float, robot_y: float, radius: float)->None:
        MAX_ITERATIONS = 100
        GOAL_SEPARATION_DISTANCE = 5.0
        DYNAMIC_GOAL_RADIUS = float(radius) if radius > GOAL_SEPARATION_DISTANCE else GOAL_SEPARATION_DISTANCE
        PREDEFINED_GOAL_LOCATIONS = [[-(ARENA_LENGTH/2 - 1), -(ARENA_WIDTH/2 - 1)], [ARENA_LENGTH/2 - 1, ARENA_WIDTH/2 - 1], [ARENA_LENGTH/2 - 1, -(ARENA_WIDTH/2 - 1)], [-(ARENA_LENGTH/2 - 1), ARENA_WIDTH/2 - 1]]
        self.prev_goal_x = self.goal_x
        self.prev_goal_y = self.goal_y
        iterations = 0
        while iterations < MAX_ITERATIONS:
            self.get_logger().info(f"Goal generation iteration: {iterations}")
            iterations += 1 # Prevent infinite loop
            if ENABLE_TRUE_RANDOM_GOALS:
                # Random goal generation within the arena
                goal_x = random.uniform(-ARENA_LENGTH/2, ARENA_LENGTH/2)
                goal_y = random.uniform(-ARENA_WIDTH/2, ARENA_WIDTH/2)
            elif ENABLE_DYNAMIC_GOALS:
                # Dynamic goal generation within a radius of the robot position
                goal_x = random.uniform(robot_x - DYNAMIC_GOAL_RADIUS, robot_x + DYNAMIC_GOAL_RADIUS)
                goal_y = random.uniform(robot_y - DYNAMIC_GOAL_RADIUS, robot_y + DYNAMIC_GOAL_RADIUS)
            else:
                # Get the goal from the predefined list
                index = random.randint(0, len(PREDEFINED_GOAL_LOCATIONS) - 1)
                goal_x = PREDEFINED_GOAL_LOCATIONS[index][0]
                goal_y = PREDEFINED_GOAL_LOCATIONS[index][1]

            # Check if the goal is valid and far enough from the previous goal
            if self.goal_is_valid(goal_x, goal_y) and math.sqrt((goal_x - self.prev_goal_x)**2 + (goal_y - self.prev_goal_y)**2) > GOAL_SEPARATION_DISTANCE:
                    break
            else:
                continue 
        if iterations >= MAX_ITERATIONS:
            self.get_logger().info("Goal generation failed default to 0, 0")
            goal_x = 0.0 # Default goal
            goal_y = 0.0 # Default goal
        # Set the goal pose
        self.goal_x = goal_x
        self.goal_y = goal_y

def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)

    drl_gazebo.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
