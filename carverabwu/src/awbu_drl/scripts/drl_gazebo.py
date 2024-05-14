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
from gazebo_msgs.srv import SetEntityState
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
from settings.constparams import TOPIC_SCAN, TOPIC_VELO, TOPIC_ODOM, TOPIC_CLOCK, TOPIC_OBSTACLES_ODOM,\
                                 LIDAR_DISTANCE_CAP, THRESHOLD_COLLISION, THREHSOLD_GOAL,\
                                 ENABLE_MOTOR_NOISE
# Simulation Environment Settings
# Arena dimensions
from settings.constparams import ARENA_LENGTH, ARENA_WIDTH
# Obstacle settings
from settings.constparams import MAX_NUMBER_OBSTACLES, DYNAMIC_GOAL_SEPARATION_DISTANCE_INIT 
# General
from settings.constparams import EPISODE_TIMEOUT_SECONDS, SPEED_LINEAR_MAX, SPEED_ANGULAR_MAX,\
                                 LINEAR_VELOCITY_LOC, ANGULAR_VELOCITY_LOC

# DRL ALGORITHM SETTINGS
from settings.constparams import UNKNOWN, SUCCESS, COLLISION, TIMEOUT, TUMBLE

# Robot specific settings
from settings.constparams import NUM_SCAN_SAMPLES

from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from awbu_interfaces.srv import DrlStep, EnvReady, ObstacleStart
from awbu_interfaces.msg import Obstacle

from drlutils_reward import Reward

from env_utils import GoalManager, Robot, bcolors

MAX_GOAL_DISTANCE = math.sqrt(ARENA_LENGTH**2 + ARENA_WIDTH**2)
REST_SIMULATION_PAUSE = 0.01  # seconds

PATH_INTERVAL_PER_EPISODE = 2

MAX_OBS_SPEED_X = (ARENA_LENGTH / EPISODE_TIMEOUT_SECONDS) * PATH_INTERVAL_PER_EPISODE
MAX_OBS_SPEED_Y = (ARENA_WIDTH / EPISODE_TIMEOUT_SECONDS) * PATH_INTERVAL_PER_EPISODE

from env_utils import read_stage

STAGE = read_stage()
if STAGE == 1:
    INTIAL_ROBOT_X = 0.0
    INTIAL_ROBOT_Y = 0.0
elif STAGE == 2:
    INTIAL_ROBOT_X = -8.0
    INTIAL_ROBOT_Y = 0.0
else:
    INTIAL_ROBOT_X = 0.0
    INTIAL_ROBOT_Y = 0.0


class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        '''
        
        Goal Box entity
        
        '''

        self.entity_name = 'goal_area'

        '''
        
        Initialize variables
        
        '''
        
        # --------------- Robot --------------- #
        self.robot = Robot(initial_x=INTIAL_ROBOT_X, initial_y=INTIAL_ROBOT_Y)
        # --------------- Goal --------------- #
        self.goal_ready = False
        self.goal_x, self.goal_y = 0.0, 0.0

        self.obstacle_pos_x = self.robot.x + LIDAR_DISTANCE_CAP     # meters
        self.obstacle_pos_y = self.robot.y + LIDAR_DISTANCE_CAP     # meters

        self.obstacle_vel_x = 0.0
        self.obstacle_vel_y = 0.0

        # --------------- Laser Scanner --------------- #
        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance_nearest = LIDAR_DISTANCE_CAP
        self.obstacle_distances = [np.inf] * MAX_NUMBER_OBSTACLES

        # --------------- Time and Episode --------------- #
        self.episode_timeout = EPISODE_TIMEOUT_SECONDS
        self.time_sec = 0
        self.episode_start_time = 0
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
        
        Manager
        
        '''
        self.goal_manager = GoalManager()
        self._dynamic_goals_radius = DYNAMIC_GOAL_SEPARATION_DISTANCE_INIT
        self._dynamic_goals_reset_flag = True
        self.reward_manager = Reward()


        '''
        
        Initialize Node
        
        '''

        # Initialise publishers
        self.cmd_vel_pub                = self.create_publisher(Twist, TOPIC_VELO, qos)

        # subscribers
        self.odom_sub                   = self.create_subscription(Odometry, TOPIC_ODOM, self.odom_callback, qos)
        self.scan_sub                   = self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.clock_sub                  = self.create_subscription(Clock, TOPIC_CLOCK, self.clock_callback, qos_profile=qos_clock)
        self.obstacle_odom_sub          = self.create_subscription(Obstacle, TOPIC_OBSTACLES_ODOM, self.obstacle_odom_callback, qos)

        # Initialise services clients
        self.delete_entity_client       = self.create_client(DeleteEntity, '/delete_entity')
        self.reset_simulation_client    = self.create_client(Empty, '/reset_world')
        self.gazebo_pause               = self.create_client(Empty, '/pause_physics')
        self.gazebo_unpause             = self.create_client(Empty, '/unpause_physics')
        self.set_entity_state_client    = self.create_client(SetEntityState, '/gazebo_drl/set_entity_state')
        self.obstacle_start_client      = self.create_client(ObstacleStart, '/obstacle_start')

        # Initialise services servers
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)
        self.goal_comm_server = self.create_service(EnvReady, 'env_comm', self.env_comm_callback)

        # Initialize Node
        self.init_drl()

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

    def set_entity_state(self, goal_x: float, goal_y: float):
        request = SetEntityState.Request()
        request.state.name = self.entity_name
        request.state.pose = Pose()
        request.state.pose.position.x = goal_x
        request.state.pose.position.y = goal_y
        request.state.twist = Twist()
        request.state.reference_frame = 'world'
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            # self.get_logger().info(f'Setting entity state for {entity_name}...')
            self.set_entity_state_client.call_async(request)
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def obstacle_start(self):
        req = ObstacleStart.Request()
        while not self.obstacle_start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.obstacle_start_client.call_async(req)

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

    def obstacle_odom_callback(self, msg: Obstacle):
        if len(msg.id) != 0:
            # Get the closest obstacle
            max_cp_loc = np.argmax(msg.cp)

            self.obstacle_pos_x = msg.pose_x[max_cp_loc]
            self.obstacle_pos_y = msg.pose_y[max_cp_loc]

            self.obstacle_vel_x = msg.velocity_x[max_cp_loc]
            self.obstacle_vel_y = msg.velocity_y[max_cp_loc]

        else:
            self.obstacle_pos_x = self.robot.x + LIDAR_DISTANCE_CAP     # meters
            self.obstacle_pos_y = self.robot.y + LIDAR_DISTANCE_CAP     # meters

            self.obstacle_vel_x = 0.0
            self.obstacle_vel_y = 0.0

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
    
    # def get_state(self,
    #     action_linear_previous: float,
    #     action_angular_previous: float
    # ):
    #     # state = copy.deepcopy(self.scan_ranges)                                                     # range: [ 0, 1]
    #     # state.append(float(np.clip((self.robot.distance_to_goal / MAX_GOAL_DISTANCE), 0, 1)))       # range: [ 0, 1]
    #     # state.append(float(self.robot.goal_angle) / math.pi)                                        # range: [-1, 1]
    #     # state.append(float(action_linear_previous))                                                 # range: [-1, 1]
    #     # state.append(float(action_angular_previous))                                                # range: [-1, 1]
    #     self.local_step += 1
    #     return state

    # def get_state(self, action_linear_previous: float, action_angular_previous: float):
    #     # Distance Obervation
    #     state = copy.deepcopy(self.scan_ranges)
    #     # Goal Related Obervation
    #     dtg = self.robot.distance_to_goal / MAX_GOAL_DISTANCE
    #     atg = self.robot.goal_angle / math.pi
    #     # Robot Observation
    #     x = self.robot.x / ARENA_LENGTH
    #     y = self.robot.y / ARENA_WIDTH
    #     theta = self.robot.theta / math.pi
    #     vel = self.robot.linear_velocity / SPEED_LINEAR_MAX
    #     omega = self.robot.angular_velocity / SPEED_ANGULAR_MAX
    #     # Obstacle Observation
    #     obstacle_x = self.obstacle_pos_x / ARENA_LENGTH
    #     obstacle_y = self.obstacle_pos_y / ARENA_WIDTH
    #     obstacle_vel_x = self.obstacle_vel_x / SPEED_LINEAR_MAX
    #     obstacle_vel_y = self.obstacle_vel_y / SPEED_LINEAR_MAX
    #     # Append the state
    #     state.append(float(dtg))
    #     state.append(float(atg))
    #     state.append(float(x))
    #     state.append(float(y))
    #     state.append(float(theta))
    #     state.append(float(vel))
    #     state.append(float(omega))
    #     state.append(float(obstacle_x))
    #     state.append(float(obstacle_y))
    #     state.append(float(obstacle_vel_x))
    #     state.append(float(obstacle_vel_y))
    #     # self.get_logger().info(f'DTG: {dtg:.2f} ATG: {atg:.2f} X: {x:.2f} Y: {y:.2f} Θ: {theta:.2f} || V: {vel:.2f} Ω: {omega:.2f}  || OX: {obstacle_x:.2f} OY: {obstacle_y:.2f} OVX: {obstacle_vel_x:.2f} OVY: {obstacle_vel_y:.2f}')
    #     # self.get_logger().info(f'State: {state}')
    #     self.local_step += 1
    #     return state
    
    def get_state(self, action_linear_previous: float, action_angular_previous: float):
        # Distance Obervation
        state = copy.deepcopy(self.scan_ranges)
        # Goal Related Obervation
        dtg = self.robot.distance_to_goal / MAX_GOAL_DISTANCE
        atg = self.robot.goal_angle / math.pi
        # Robot Observation
        theta = self.robot.theta / math.pi
        vel = self.robot.linear_velocity / SPEED_LINEAR_MAX
        omega = self.robot.angular_velocity / SPEED_ANGULAR_MAX
        # Relative Obstacle Observation
        relative_robot_obstacle_distance = np.sqrt((self.obstacle_pos_x - self.robot.x)**2 + (self.obstacle_pos_y - self.robot.y)**2) / LIDAR_DISTANCE_CAP
        relative_robot_obstacle_angle = math.atan2(self.obstacle_pos_y - self.robot.y, self.obstacle_pos_x - self.robot.x) / math.pi
        # Relative Linear and Angular velocity
        obstacle_linear_vel = np.sqrt(self.obstacle_vel_x**2 + self.obstacle_vel_y**2)
        obstacle_angular_vel = math.atan2(self.obstacle_vel_y, self.obstacle_vel_x)
        relative_robot_obstacle_linear_vel = (obstacle_linear_vel - self.robot.linear_velocity) / SPEED_LINEAR_MAX
        relative_robot_obstacle_angular_vel = (obstacle_angular_vel - self.robot.angular_velocity) / SPEED_ANGULAR_MAX

        # Append the state
        state.append(float(dtg))
        state.append(float(atg))
        state.append(float(theta))
        state.append(float(vel))
        state.append(float(omega))
        state.append(float(relative_robot_obstacle_distance))
        state.append(float(relative_robot_obstacle_angle))
        state.append(float(relative_robot_obstacle_linear_vel))
        state.append(float(relative_robot_obstacle_angular_vel))
        # self.get_logger().info(f'DTG: {dtg:.2f} ATG: {atg:.2f} X: {x:.2f} Y: {y:.2f} Θ: {theta:.2f} || V: {vel:.2f} Ω: {omega:.2f}  || OX: {obstacle_x:.2f} OY: {obstacle_y:.2f} OVX: {obstacle_vel_x:.2f} OVY: {obstacle_vel_y:.2f}')
        # self.get_logger().info(f'State: {state}')
        self.local_step += 1
        return state
    
    def episode_check(self):
        # self.get_logger().info(f"Obstacle distance: {self.obstacle_distance_nearest:.2f}")
        # Success
        if self.robot.distance_to_goal < THREHSOLD_GOAL:
            self.get_logger().info(bcolors.OKGREEN + "Episode done, Agent reached the goal!" + bcolors.ENDC)
            self._EP_succeed = SUCCESS
        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self.get_logger().info(bcolors.WARNING + "Episode done, Agent reached the timeout!" + bcolors.ENDC)
            self._EP_succeed = TIMEOUT
        # Collision
        elif self.obstacle_distance_nearest < THRESHOLD_COLLISION:
            self.get_logger().info(bcolors.FAIL + f"Episode done, Collision with obstacle: {self.obstacle_distance_nearest:.2f}" + bcolors.ENDC)
            self._EP_succeed = COLLISION
        # Tumble [row, pitch > 45°]
        elif np.abs(self.robot.row) > math.pi/4 or np.abs(self.robot.pitch) > math.pi/4:
            self.get_logger().info(bcolors.FAIL + f"Episode done, Tumble: {math.degrees(self.robot.row):.2f}, {math.degrees(self.robot.pitch):.2f}" + bcolors.ENDC)
            self._EP_succeed = TUMBLE

        # Check if the episode is done [Success, Collision, Timeout, Tumble] 
        if self._EP_succeed is not UNKNOWN:
            self.episode_done(self._EP_succeed)
    
    def episode_done(self, status: int):
        # Stop the robot 
        self.cmd_vel_pub.publish(Twist())
        # Reset the episode deadline
        self.episode_deadline = np.inf
        self._EP_done = True
        # If Dynamic goals are enabled
        if ENABLE_DYNAMIC_GOALS:
            if status == SUCCESS:
                # Increase the goal radius
                self._dynamic_goals_reset_flag = False # Do not reset the simulation
                self._dynamic_goals_radius *= 1.01
                self.get_logger().info(bcolors.OKGREEN + f"Goal reached, increasing goal radius to {self._dynamic_goals_radius:.2f}" + bcolors.ENDC)
            else:
                # Decrease the goal radius
                self._dynamic_goals_reset_flag = True # Reset the simulation
                self._dynamic_goals_radius *= 0.99
                self.get_logger().info(bcolors.FAIL + f"Goal not reached, decreasing goal radius to {self._dynamic_goals_radius:.2f}" + bcolors.ENDC)
    
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

        # Reset the simulation
        if ENABLE_DYNAMIC_GOALS:
            # If dynamic goals are enabled, reset the simulation only when episode is not successful
            if self._dynamic_goals_reset_flag:
                self.reset_simulation()
        else:
            self.reset_simulation()

        # Generate a new goal
        self.goal_x, self.goal_y = self.goal_manager.generate_goal_pose(self.robot.x, self.robot.y, self._dynamic_goals_radius)

        # Set the goal entity
        self.set_entity_state(self.goal_x, self.goal_y)

        # Clear the obstacle distances
        self.obstacle_distance_nearest = LIDAR_DISTANCE_CAP

        # Update the robot goal
        self.robot.update_goal(self.goal_x, self.goal_y)
        # Reset the robot
        self.robot.reset()

        # Start the robot
        self.cmd_vel_pub.publish(Twist())

        # Start the obstacles
        self.obstacle_start()

        # Reset the episode variables
        self.reset_deadline = True

        response.state = self.get_state(action_linear_previous=0.0, action_angular_previous=0.0)
        response.reward = 0.0
        response.done = False
        response.success = UNKNOWN
        response.distance_traveled = 0.0
        
        self.get_logger().info(f"=====================================")
        # self.get_logger().info(bcolors.OKBLUE + f"New episode started, Goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}" + bcolors.ENDC)
        self.get_logger().info(bcolors.OKBLUE + f"Goal location: ({self.goal_x:.2f}, {self.goal_y:.2f}) DTG: {self.robot.distance_to_goal:.2f} AG: {math.degrees(self.robot.goal_angle):.1f}°" + bcolors.ENDC)

        self.reward_manager.reward_initalize(self.robot.distance_to_goal / MAX_GOAL_DISTANCE, self.robot.goal_angle / math.pi)

        # Unpause the simulation
        self.episode_start_time = self.time_sec
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
        self.cmd_vel_pub.publish(twist)

        # Prepare repsonse
        response.state = self.get_state(
            action_linear_previous  = request.previous_action[LINEAR_VELOCITY_LOC],
            action_angular_previous = request.previous_action[ANGULAR_VELOCITY_LOC])
        # Check if the episode is done
        self.episode_check()
        # Calculate reward
        reward_out, [R_DISTANCE, R_ANGLE, R_WAYPOINT, R_OBSTACLE] = self.reward_manager.get_reward(
            status              = self._EP_succeed,
            action_linear       = action_linear, # not used
            action_angular      = action_angular, # not used
            distance_to_goal    = self.robot.distance_to_goal / MAX_GOAL_DISTANCE, # Normalize the distance
            angle_to_goal       = self.robot.goal_angle / math.pi, # Normalize the angle
            omega               = action_angular, # not used
            min_obstacle_dist   = self.obstacle_distance_nearest # the obstacle distance
        )
        if R_OBSTACLE != 0:
            self.get_logger().info(bcolors.WARNING + f"Obstacle detected at {self.obstacle_distance_nearest:.2f} meters, reward: {reward_out:.2f}, DTG: {self.robot.distance_to_goal:.2f} AG: {math.degrees(self.robot.goal_angle):.1f}°" + bcolors.ENDC)
        if R_WAYPOINT != 0:
            self.get_logger().info(bcolors.OKCYAN+ f"Waypoint reached at {self.robot.distance_to_goal:.2f} meters, reward: {reward_out:.2f}, DTG: {self.robot.distance_to_goal:.2f} AG: {math.degrees(self.robot.goal_angle):.1f}°" + bcolors.ENDC)
        response.reward = reward_out
        response.done = self._EP_done
        response.success = self._EP_succeed
        response.distance_traveled = 0.0
        # Check if the episode is done
        if self._EP_done:
            self.get_logger().info(f"Reward: {response.reward:<8.2f} DTG: {self.robot.distance_to_goal:<8.2f}AG: {math.degrees(self.robot.goal_angle):.1f}°")
            self.get_logger().info(f'Time use in episode: {self.time_sec - self.episode_start_time} seconds / {self.episode_timeout} seconds')
            response.distance_traveled = self.robot.distance_traveled
            # Reset variables
            self._EP_succeed = UNKNOWN
            self.local_step = 0
            self._EP_done = False
        if self.local_step % 2 == 0:
            print(f"T: {self.time_sec:<8}RT:{self.real_node_time_sec:<8}EPD: {self.episode_deadline:<8}\t")
            print(f"Reward: {response.reward:<8.2f}DTG: {self.robot.distance_to_goal:<8.2f}AG: {math.degrees(self.robot.goal_angle):.1f}°\t")
            print(f"MinOBD: {self.obstacle_distance_nearest:<8.2f}Alin: {request.action[LINEAR_VELOCITY_LOC]:<7.1f}Aturn: {request.action[ANGULAR_VELOCITY_LOC]:<7.1f}")
        return response

    '''

    DRL Genral functions

    '''

    def init_drl(self)->None:
        # Initialize the DRL node
        self.pause_simulation()
        self.reset_simulation() # Reset the simulation
        self.cmd_vel_pub.publish(Twist()) # Stop the robot if it is moving
        self.unpause_simulation() 
        self.get_logger().info(bcolors.OKCYAN + "DRL Gazebo node has been initialized, Simulation Paused" + bcolors.ENDC)
        self.get_logger().info(bcolors.OKGREEN + f"Please start the episode by calling the service..." + bcolors.ENDC)
        # Goal is ready
        self.goal_ready = True

def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)

    drl_gazebo.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
