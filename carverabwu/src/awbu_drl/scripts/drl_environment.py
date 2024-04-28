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

import math
import numpy
import sys
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from awbu_interfaces.srv import DrlStep, Goal, RingGoal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy

import reward as rw
from common import utilities as util

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

# Automatically retrievew from Gazebo model configuration (40 by default).
# Can be set manually if needed.

MAX_GOAL_DISTANCE = math.sqrt(ARENA_LENGTH**2 + ARENA_WIDTH**2)
class DRLEnvironment(Node):
    def __init__(self):
        super().__init__('drl_environment')
        # ------------ Initialize variables ----------
        # Episode
        self.episode_timeout = EPISODE_TIMEOUT_SECONDS
        # Topics
        self.scan_topic = TOPIC_SCAN
        self.velo_topic = TOPIC_VELO
        self.odom_topic = TOPIC_ODOM
        self.goal_topic = TOPIC_GOAL
        # Robot variables
        self.goal_x, self.goal_y = 0.0, 0.0 # goal position
        self.robot_x, self.robot_y = 0.0, 0.0 # robot position
        self.robot_x_prev, self.robot_y_prev = 0.0, 0.0 # robot previous position
        self.robot_heading = 0.0 # robot heading angle
        self.robot_tilt = 0.0 # robot orientation angle
        self.total_distance = 0.0 # total distance traveled
        
        # Episode variables
        self.done = False
        self.succeed = UNKNOWN
        self.episode_deadline = np.inf
        self.reset_deadline = False
        self.clock_msgs_skipped = 0

        self.obstacle_distances = [np.inf] * MAX_NUMBER_OBSTACLES

        self.new_goal = False
        self.goal_angle = 0.0
        self.goal_distance = MAX_GOAL_DISTANCE
        self.initial_distance_to_goal = MAX_GOAL_DISTANCE

        self.scan_ranges = [LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance = LIDAR_DISTANCE_CAP

        self.difficulty_radius = 1
        self.local_step = 0
        self.time_sec = 0

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        # publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.velo_topic, qos)
        # subscribers
        self.goal_pose_sub = self.create_subscription(Pose, self.goal_topic, self.goal_pose_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile=qos_clock)
        self.obstacle_odom_sub = self.create_subscription(Odometry, 'obstacle/odom', self.obstacle_odom_callback, qos)
        # clients
        self.task_succeed_client = self.create_client(RingGoal, 'task_succeed')
        self.task_fail_client = self.create_client(RingGoal, 'task_fail')
        # servers
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)
        self.goal_comm_server = self.create_service(Goal, 'goal_comm', self.goal_comm_callback)

        # Timer
        self.create_timer(1.0, self.timer_callback)
        self.temp_i = 0 
    
    def timer_callback(self):
        self.get_logger().info(f"Gazebo Time: {self.time_sec} Node Time: {self.temp_i}")
        # self.get_logger().info(f"Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}) tilt: {self.robot_tilt:.2f}")
        # self.get_logger().info(f"Goal Distance: {self.goal_distance:.2f} | Goal Angle: {math.degrees(self.goal_angle):.2f}")
        # self.get_logger().info(f"State: {self.get_state(self.temp_i, self.temp_i)}")
        self.temp_i += 1

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def goal_pose_callback(self, msg: Pose):
        self.goal_x = msg.position.x
        self.goal_y = msg.position.y
        self.new_goal = True
        print(f"new goal! x: {self.goal_x} y: {self.goal_y}")

    def goal_comm_callback(self, request: Goal.Request, response: Goal.Response):
        response.new_goal = self.new_goal
        return response

    def obstacle_odom_callback(self, msg: Odometry):
        if 'obstacle' in msg.child_frame_id:
            robot_pos = msg.pose.pose.position
            obstacle_id = int(msg.child_frame_id[-1]) - 1
            diff_x = self.robot_x - robot_pos.x
            diff_y = self.robot_y - robot_pos.y
            self.obstacle_distances[obstacle_id] = math.sqrt(diff_y**2 + diff_x**2)
        else:
            print("ERROR: received odom was not from obstacle!")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = util.euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_tilt = msg.pose.pose.orientation.y

        # calculate traveled distance for logging
        if self.local_step % 32 == 0:
            self.total_distance += math.sqrt(
                (self.robot_x_prev - self.robot_x)**2 +
                (self.robot_y_prev - self.robot_y)**2)
            self.robot_x_prev = self.robot_x
            self.robot_y_prev = self.robot_y

        diff_y = self.goal_y - self.robot_y
        diff_x = self.goal_x - self.robot_x
        distance_to_goal = math.sqrt(diff_x**2 + diff_y**2)
        heading_to_goal = math.atan2(diff_y, diff_x)
        goal_angle = heading_to_goal - self.robot_heading

        while goal_angle > math.pi:
            goal_angle -= 2 * math.pi
        while goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = distance_to_goal
        self.goal_angle = goal_angle

    def scan_callback(self, msg: LaserScan):
        if len(msg.ranges) != NUM_SCAN_SAMPLES:
            print(f"more or less scans than expected! check model.sdf, got: {len(msg.ranges)}, expected: {NUM_SCAN_SAMPLES}")
        # normalize laser values
        self.obstacle_distance = 1
        for i in range(NUM_SCAN_SAMPLES):
                self.scan_ranges[i] = numpy.clip(float(msg.ranges[i]) / LIDAR_DISTANCE_CAP, 0, 1)
                if self.scan_ranges[i] < self.obstacle_distance:
                    self.obstacle_distance = self.scan_ranges[i]
        self.obstacle_distance *= LIDAR_DISTANCE_CAP

    def clock_callback(self, msg: Clock):
        self.get_logger().info(f"Clock: {msg.clock.sec}")
        # Get current time
        self.time_sec = msg.clock.sec
        # Reset episode deadline
        if not self.reset_deadline:
            self.get_logger().info("Not resetting deadline")
            return
        # Skip first few messages to avoid resetting clock before simulation is ready
        self.clock_msgs_skipped += 1
        if self.clock_msgs_skipped <= 10: # Wait a few message for simulation to reset clock
            return
        # Reset episode deadline
        episode_time = self.episode_timeout
        if ENABLE_DYNAMIC_GOALS:
            episode_time = numpy.clip(episode_time * self.difficulty_radius, 10, 50)
        # Set deadline
        self.get_logger().info(f"Resetting episode deadline to {episode_time} seconds")
        self.episode_deadline = self.time_sec + episode_time
        self.get_logger().info(f"Episode deadline set to {self.episode_deadline} seconds")
        # Reset variables
        self.reset_deadline = False
        self.clock_msgs_skipped = 0

    def stop_reset_robot(self, success: bool):
        self.cmd_vel_pub.publish(Twist()) # stop robot
        self.episode_deadline = np.inf
        self.done = True
        req = RingGoal.Request()
        req.robot_pose_x = self.robot_x
        req.robot_pose_y = self.robot_y
        req.radius = numpy.clip(self.difficulty_radius, 0.5, 4)
        if success:
            self.difficulty_radius *= 1.01
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('success service not available, waiting again...')
            self.task_succeed_client.call_async(req)
        else:
            self.difficulty_radius *= 0.99
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('fail service not available, waiting again...')
            self.task_fail_client.call_async(req)

    def get_state(self, action_linear_previous: float, action_angular_previous: float):
        state = copy.deepcopy(self.scan_ranges)                                             # range: [ 0, 1]
        state.append(float(numpy.clip((self.goal_distance / MAX_GOAL_DISTANCE), 0, 1)))     # range: [ 0, 1]
        state.append(float(self.goal_angle) / math.pi)                                      # range: [-1, 1]
        state.append(float(action_linear_previous))                                         # range: [-1, 1]
        state.append(float(action_angular_previous))                                        # range: [-1, 1]
        self.local_step += 1

        if self.local_step <= 30: # Grace period to wait for simulation reset
            return state
    
        # self.get_logger().info(f"Time: {self.time_sec} | Episode Deadline: {self.episode_deadline} | Episode Timeout: {self.episode_timeout}")

        # Success
        if self.goal_distance < THREHSOLD_GOAL:
            self.succeed = SUCCESS
        # Collision
        elif self.obstacle_distance < THRESHOLD_COLLISION:
            dynamic_collision = False
            for obstacle_distance in self.obstacle_distances:
                if obstacle_distance < (THRESHOLD_COLLISION + OBSTACLE_RADIUS + 0.05):
                    dynamic_collision = True
            if dynamic_collision:
                self.succeed = COLLISION_OBSTACLE
            else:
                self.succeed = COLLISION_WALL
        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self.succeed = TIMEOUT
        # Tumble
        elif self.robot_tilt > 0.06 or self.robot_tilt < -0.06:
            self.succeed = TUMBLE
        if self.succeed is not UNKNOWN:
            self.stop_reset_robot(self.succeed == SUCCESS)
        return state

    def initalize_episode(self, response: DrlStep.Response):
        self.initial_distance_to_goal = self.goal_distance
        response.state = self.get_state(0, 0)
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0
        rw.reward_initalize(self.initial_distance_to_goal)
        return response

    def step_comm_callback(self, request: DrlStep.Request, response: DrlStep.Response):

        # self.get_logger().info(f"Step {self.local_step} | Agent: {request.action} | Previous: {request.previous_action}")

        if len(request.action) == 0:
            return self.initalize_episode(response)

        if ENABLE_MOTOR_NOISE:
            request.action[LINEAR_VELOCITY_LOC] += numpy.clip(numpy.random.normal(0, 0.05), -0.1, 0.1)
            request.action[ANGULAR_VELOCITY_LOC] += numpy.clip(numpy.random.normal(0, 0.05), -0.1, 0.1)

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
        response.state = self.get_state(request.previous_action[LINEAR_VELOCITY_LOC], request.previous_action[ANGULAR_VELOCITY_LOC])
        response.reward = rw.get_reward(self.succeed, action_linear, action_angular, self.goal_distance,
                                            self.goal_angle, self.obstacle_distance)
        response.done = self.done
        response.success = self.succeed
        response.distance_traveled = 0.0
        if self.done:
            self.get_logger().info(f"Episode done! Reward: {response.reward:.2f} Success: {self.succeed}")
            response.distance_traveled = self.total_distance
            # Reset variables
            self.succeed = UNKNOWN
            self.total_distance = 0.0
            self.local_step = 0
            self.done = False
            self.reset_deadline = True
        if self.local_step % 200 == 0:
            print(f"Rtot: {response.reward:<8.2f}GD: {self.goal_distance:<8.2f}GA: {math.degrees(self.goal_angle):.1f}°\t", end='')
            print(f"MinD: {self.obstacle_distance:<8.2f}Alin: {request.action[LINEAR_VELOCITY_LOC]:<7.1f}Aturn: {request.action[ANGULAR_VELOCITY_LOC]:<7.1f}")
        return response

def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    if len(args) == 0:
        drl_environment = DRLEnvironment()
    else:
        rclpy.shutdown()
        quit("ERROR: wrong number of arguments!")
    rclpy.spin(drl_environment)
    drl_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
