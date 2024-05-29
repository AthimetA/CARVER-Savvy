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

from settings.constparams import REWARD_FUNCTION, COLLISION, TUMBLE, SUCCESS, TIMEOUT,\
      SPEED_LINEAR_MAX, THRESHOLD_COLLISION, SPEED_ANGULAR_MAX, NUM_SCAN_SAMPLES
import numpy as np
from env_utils import get_simulation_speed, read_stage, bcolors

class Reward():
    def __init__(self):
        self.distance_to_goal = 0
        self.angle_to_goal = 0

        self.initial_distance_to_goal = 0
        self.initial_angle_to_goal = 0
        self.waypoint_list = []
        self.waypoint_idx = 0
        self.waypoint_reached = False

        self.action_linear_prev = 0
        self.action_angular_prev = 0

    def reward_initalize(self, init_distance_to_goal, init_angle_to_goal):
        # Update the initial distance and angle to goal
        self.distance_to_goal = init_distance_to_goal
        self.angle_to_goal = init_angle_to_goal

        self.initial_distance_to_goal = init_distance_to_goal
        self.initial_angle_to_goal = init_angle_to_goal

        self.action_linear_prev = 0
        self.action_angular_prev = 0

        # Calculate the waypoints (initial distance to goal / 5) in reverse order
        self.waypoint_list = [init_distance_to_goal - i * (init_distance_to_goal / 5) for i in range(1, 5)]
        self.waypoint_idx = 0
        self.waypoint_reached = False

    def get_reward(self,
    status,  # Status of the robot (SUCCESS, COLLISION, TUMBLE, TIMEOUT)
    action_linear,  # Linear velocity
    action_angular,  # Angular velocity
    distance_to_goal,  # Distance to the goal
    angle_to_goal,  # Angle to the goal
    omega,  # Angular velocity
    scan_ranges,  # lidar scan
    ):
        SCALING_FACTOR = 2.0
        
        # Step reward for each action
        R_STEP = - 2
        # Reward for the angle to the goal
        # [-1, 0] # Angle to the goal (Normalized by 1/3.14)
        R_ANGLE = (-1 * abs(angle_to_goal) / np.pi) * SCALING_FACTOR

        # Reward for the distance to the goal
        # [-1, 0] # Distance to the goal(Normalized by MAX_DISTANCE)
        R_DISTANCE = -1 * abs(distance_to_goal) * SCALING_FACTOR

        # Reward for the angular velocity
        # Penalty for angular velocity to prevent spinning in place and lower the angular velocity
        # [-1, 0]
        R_ANGULAR = -1 * np.abs(action_angular) * SCALING_FACTOR
        
        # Reward for the linear velocity
        # Penalty for linear velocity to prevent high speed (Since the robot is [-1, 1] and -1 is 0 m/s)
        # [-1, 0]
        R_LINEAR = -1 * (action_linear + 1)/2 * SCALING_FACTOR # +1 to make it [0, 2] and divide by 2 to make it [0, 1]

        # Waypoint reward
        R_WAYPOINT = 0
        if not self.waypoint_reached:
            
            # If the distance to the goal is less than the waypoint distance
            if distance_to_goal < self.waypoint_list[self.waypoint_idx]:
                self.waypoint_idx += 1 # Move to the next waypoint
                R_WAYPOINT = 100

            # If the last waypoint is reached
            if self.waypoint_idx == 4:
                self.waypoint_reached = True

        # Reward for the scan
        R_FONT_SCAN, R_OTHER_SCAN = self.get_reward_from_scan(scan_ranges)

        # Reward for status
        if status == SUCCESS:
            R_STATUS = 250
        elif status == COLLISION:
            R_STATUS = -250
        elif status == TIMEOUT:
            R_STATUS = -250
        else:
            R_STATUS = 0

        # Total reward
        reward = R_STATUS + R_STEP + R_ANGLE + R_DISTANCE + R_WAYPOINT


        return float(reward) , [R_DISTANCE, R_ANGLE, R_WAYPOINT, R_FONT_SCAN, R_OTHER_SCAN]
    
    def get_reward_from_scan(self, scan_ranges):

        # R_FONT_SCAN + R_OTHER_SCAN should be in the range of [-4, 0] 
        # The Threshold is R_FONT_SCAN = -3, R_OTHER_SCAN = -1

        DEG_PER_SCAN = 360 / NUM_SCAN_SAMPLES

        font_angle_fov = 60 # degrees

        front_scan_start_index = int((180 - font_angle_fov) / DEG_PER_SCAN)
        front_scan_end_index = int((180 + font_angle_fov) / DEG_PER_SCAN)

        front_scan = np.asarray(scan_ranges[front_scan_start_index:front_scan_end_index])

        R_FONT_SCAN_MAX = len(front_scan) # Maximum value of R_FONT_SCAN --> all the values are -1

        R_FONT_SCAN = np.sum((front_scan - 1.0)) / R_FONT_SCAN_MAX

        other_scan = np.asarray(scan_ranges[:front_scan_start_index] + scan_ranges[front_scan_end_index:])

        R_OTHER_SCAN_MAX = len(other_scan) # Maximum value of R_OTHER_SCAN --> all the values are -1

        R_OTHER_SCAN = np.sum((other_scan - 1.0)) / R_OTHER_SCAN_MAX

        # Scaling the reward

        R_FONT_SCAN = R_FONT_SCAN * 1

        R_OTHER_SCAN = R_OTHER_SCAN * 0.5

        return R_FONT_SCAN, R_OTHER_SCAN