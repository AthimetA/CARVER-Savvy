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

import math
import copy
import numpy as np
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from awbu_interfaces.srv import DrlStep, EnvReady, UserSetGoal
from awbu_interfaces.msg import Obstacle

from drlutils_reward import Reward
from env_utils import Robot, bcolors


# ENVIRONMENT SETTINGS 
from settings.constparams import TOPIC_SCAN, TOPIC_VELO, TOPIC_REAL_ODOM, TOPIC_OBSTACLES_ODOM
from settings.constparams import SRV_RESET_OBSTACLES_CP, SRV_ENV_COMM, SRV_STEP_COMM, SRV_USER_SET_GOAL

from settings.constparams import REAL_LIDAR_DISTANCE_CAP, REAL_THRESHOLD_COLLISION, REAL_THRESHOLD_GOAL

from settings.constparams import REAL_SPEED_LINEAR_MAX, REAL_SPEED_ANGULAR_MAX, LINEAR_VELOCITY_LOC, ANGULAR_VELOCITY_LOC                            

# DRL ALGORITHM SETTINGS
from settings.constparams import UNKNOWN, SUCCESS, COLLISION, TIMEOUT, TUMBLE, REAL_EPISODE_TIMEOUT_SECONDS

# Robot specific settings
from settings.constparams import NUM_SCAN_SAMPLES

from drlutils_visual_observation import VisualObservation
import PyQt5.QtWidgets as QtWidgets

MAX_GOAL_DISTANCE = 10.0 # meters

MAX_OBS_SPEED_X = 2.0 # m/s
MAX_OBS_SPEED_Y = 2.0 # m/s

INTIAL_ROBOT_X = 0
INTIAL_ROBOT_Y = 0

STATE_SIZE = NUM_SCAN_SAMPLES + 2 + 6 + 4 + 2
HIDDEN_SIZE = 256

class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        '''
        
        Initialize variables
        
        '''
        self.env_ready = False
        
        # --------------- Robot --------------- #
        self.robot = Robot(initial_x=INTIAL_ROBOT_X, initial_y=INTIAL_ROBOT_Y)
        # --------------- Goal --------------- #
        self.goal_x, self.goal_y = 0.0, 0.0

        self.obstacle_pos_x = self.robot.x     # meters
        self.obstacle_pos_y = self.robot.y     # meters

        self.obstacle_vel_x = 0.0
        self.obstacle_vel_y = 0.0

        # --------------- Laser Scanner --------------- #
        self.scan_ranges = [REAL_LIDAR_DISTANCE_CAP] * NUM_SCAN_SAMPLES
        self.obstacle_distance_nearest = REAL_LIDAR_DISTANCE_CAP
        # --------------- ROS Parameters --------------- #
        qos = QoSProfile(depth=10)
        '''
        
        Manager
        
        '''
        self.reward_manager = Reward()


        '''
        
        Initialize Node
        
        '''

        # Initialise publishers
        self.cmd_vel_pub                = self.create_publisher(Twist, TOPIC_VELO, qos)

        # subscribers
        self.odom_sub                   = self.create_subscription(Odometry, TOPIC_REAL_ODOM, self.odom_callback, qos_profile=qos)
        self.scan_sub                   = self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.obstacle_odom_sub          = self.create_subscription(Obstacle, TOPIC_OBSTACLES_ODOM, self.obstacle_odom_callback, qos_profile=qos)

        # Initialise services clients
        self.obstacle_cp_reset_client   = self.create_client(Empty, SRV_RESET_OBSTACLES_CP)

        # Initialise services servers
        self.step_comm_server = self.create_service(DrlStep, SRV_STEP_COMM, self.step_comm_callback)
        self.env_comm_server = self.create_service(EnvReady, SRV_ENV_COMM, self.env_comm_callback)
        self.user_set_goal_server = self.create_service(UserSetGoal, SRV_USER_SET_GOAL, self.user_set_goal_callback)

        # Initialize the DRL node
        self.cmd_vel_pub.publish(Twist()) # Stop the robot if it is moving
        self.get_logger().info(bcolors.OKCYAN + "DRL Gazebo node has been initialized, Simulation Paused" + bcolors.ENDC)
        self.get_logger().info(bcolors.OKGREEN + f"Please start the episode by calling the service... /{SRV_USER_SET_GOAL}" + bcolors.ENDC)

        # --------------- Time and Episode --------------- #
        self.episode_timeout = REAL_EPISODE_TIMEOUT_SECONDS
        self.time_sec = 0
        self.episode_start_time = 0
        # Episode variables
        self.episode_deadline = np.inf
        self.reset_deadline = False

        self._EP_done = False
        self._EP_succeed = UNKNOWN
        self.local_step = 0 # Local step counter

        # Timer for the node
        self.__timer_period = 0.03333
        self.timer = self.create_timer(self.__timer_period, self.node_clock_callback)


        # Visualize the environment
        self.qapp = QtWidgets.QApplication([])
        self.visual_observation = VisualObservation(state_size=STATE_SIZE)

    '''
    
    Client functions
    
    '''

    def reset_obstacle_cp(self):
        self.get_logger().info('Resetting obstacle CP...')
        while not self.obstacle_cp_reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.obstacle_cp_reset_client.call_async(Empty.Request())


    '''
    
    Callbacks Functions
    
    '''
    def node_clock_callback(self):

        # Update the visual observation
        self.visual_observation.tab_state_update(
            states=self.get_state(action_linear_previous=0.0, action_angular_previous=0.0),
        )

        QtWidgets.QApplication.processEvents()

        if self.env_ready is False:
            self.cmd_vel_pub.publish(Twist())
        # Get current time
        self.time_sec += self.__timer_period
        # Reset episode deadline
        if not self.reset_deadline:
            return
        # Reset episode deadline
        episode_time = self.episode_timeout
        # Set deadline
        self.episode_deadline = self.time_sec + episode_time
        # Reset variables
        self.reset_deadline = False

    def odom_callback(self, msg: Odometry):
        # Update the robot position
        self.robot.update_position(msg)

    def scan_callback(self, msg: LaserScan):
        

        if len(msg.ranges) != NUM_SCAN_SAMPLES:
            # Downsample the scan data
            scan_step = len(msg.ranges) // NUM_SCAN_SAMPLES
            scandata = msg.ranges[::scan_step]

        else:
            scandata = msg.ranges
        # Replace NaN and Inf values with the maximum distance
        scandata = [REAL_LIDAR_DISTANCE_CAP if math.isnan(x) or math.isinf(x) or x > REAL_LIDAR_DISTANCE_CAP else x for x in scandata]

        # Offset the scan data
        scandata = np.asanyarray(scandata) - 0.25

        # normalize laser values
        self.obstacle_distance_nearest = 1
        for i in range(NUM_SCAN_SAMPLES):
                # Normalize the scan values
                self.scan_ranges[i] = np.clip(float(scandata[i]) / REAL_LIDAR_DISTANCE_CAP, 0, 1)
                # Check for obstacles
                if self.scan_ranges[i] < self.obstacle_distance_nearest:
                    self.obstacle_distance_nearest = self.scan_ranges[i]
        # Scale the obstacle distance
        self.obstacle_distance_nearest *= REAL_LIDAR_DISTANCE_CAP

    def obstacle_odom_callback(self, msg: Obstacle):
        if len(msg.id) != 0:
            # Get the closest obstacle
            max_cp_loc = np.argmax(msg.cp)

            msg_pose_x = msg.pose_x[max_cp_loc] / REAL_LIDAR_DISTANCE_CAP
            msg_pose_y = msg.pose_y[max_cp_loc] / REAL_LIDAR_DISTANCE_CAP

            msg_velocity_x = msg.velocity_x[max_cp_loc] / MAX_OBS_SPEED_X
            msg_velocity_y = msg.velocity_y[max_cp_loc] / MAX_OBS_SPEED_Y

            # Check if the obstacle is out of range
            if np.abs(msg_pose_x) > 1 or np.abs(msg_pose_y) > 1:

                self.obstacle_pos_x = self.robot.x / REAL_LIDAR_DISTANCE_CAP
                self.obstacle_pos_y = self.robot.y / REAL_LIDAR_DISTANCE_CAP

                self.obstacle_vel_x = 0.0
                self.obstacle_vel_y = 0.0

                # self.get_logger().info(bcolors.FAIL + f"Value out of range, setting position: ({self.obstacle_pos_x:.2f}, {self.obstacle_pos_y:.2f})" + bcolors.ENDC)
            
            else:
                # Update the obstacle position
                self.obstacle_pos_x = msg_pose_x
                self.obstacle_pos_y = msg_pose_y

                self.obstacle_vel_x = msg_velocity_x
                self.obstacle_vel_y = msg_velocity_y

                # self.get_logger().info(bcolors.OKGREEN + f"Obstacle found, setting position: ({self.obstacle_pos_x:.2f}, {self.obstacle_pos_y:.2f})" + bcolors.ENDC)

        else:

            self.obstacle_pos_x = self.robot.x / REAL_LIDAR_DISTANCE_CAP
            self.obstacle_pos_y = self.robot.y / REAL_LIDAR_DISTANCE_CAP

            self.obstacle_vel_x = 0.0
            self.obstacle_vel_y = 0.0

            # self.get_logger().info(bcolors.FAIL + f"No obstacle found, setting position: ({self.obstacle_pos_x:.2f}, {self.obstacle_pos_y:.2f})" + bcolors.ENDC)
    '''
    
    Service functions
    
    '''
    # if the environment is ready for agent to start
    def env_comm_callback(self, request: EnvReady.Request, response: EnvReady.Response):
        response.env_status = self.env_ready
        return response
    
    # if the user wants to set a goal and start the episode
    def user_set_goal_callback(self, request: UserSetGoal.Request, response: UserSetGoal.Response):
        # Set the goal
        self.goal_x = request.goal_pose_x
        self.goal_y = request.goal_pose_y
   
        # Reset the obstacles
        self.reset_obstacle_cp()

        # Clear the obstacle distances
        self.obstacle_distance_nearest = REAL_LIDAR_DISTANCE_CAP

        # Update the robot goal
        self.robot.update_goal(self.goal_x, self.goal_y)

        # Start the robot
        self.cmd_vel_pub.publish(Twist())

        # Reset the episode variables
        self.reset_deadline = True
        
        self.get_logger().info(f"=====================================")
        self.get_logger().info(bcolors.OKBLUE + f"Goal location: ({self.goal_x:.2f}, {self.goal_y:.2f}) DTG: {self.robot.distance_to_goal:.2f} AG: {math.degrees(self.robot.goal_angle):.1f}°" + bcolors.ENDC)

        self.reward_manager.reward_initalize(self.robot.distance_to_goal / MAX_GOAL_DISTANCE, self.robot.goal_angle / math.pi)

        # Unpause the simulation
        self.episode_start_time = self.time_sec

        # Environment is ready
        self.env_ready = True

        # Set the response
        response.status = True

        return response
    
    def get_state(self, action_linear_previous: float, action_angular_previous: float):
        # Distance Obervation
        state = copy.deepcopy(self.scan_ranges)
        # Goal Related Obervation
        dtg = self.robot.distance_to_goal / REAL_LIDAR_DISTANCE_CAP
        atg = self.robot.goal_angle
        # Robot Observation
        # X and Y components of the robot
        x = self.robot.x / REAL_LIDAR_DISTANCE_CAP
        y = self.robot.y / REAL_LIDAR_DISTANCE_CAP
        # Calculate the velocity components
        _vel = self.robot.linear_velocity / REAL_SPEED_LINEAR_MAX
        # Correcting the velocity components calculation
        Vx = _vel * math.cos(self.robot.theta)
        Vy = _vel * math.sin(self.robot.theta)
        # Angular Components of the robot
        theta = self.robot.theta
        omega = self.robot.angular_velocity / REAL_SPEED_ANGULAR_MAX
        # Obstacle Observation
        obstacle_x = self.obstacle_pos_x # Already normalized
        obstacle_y = self.obstacle_pos_y # Already normalized
        obstacle_vel_x = self.obstacle_vel_x # Already normalized
        obstacle_vel_y = self.obstacle_vel_y # Already normalized

        # Append the state
        # Goal Related Obervation
        state.append(float(dtg))
        state.append(float(atg))
        # Robot Observation
        state.append(float(x))
        state.append(float(y))
        state.append(float(theta))
        state.append(float(Vx))
        state.append(float(Vy))
        state.append(float(omega))
        # Obstacle observation
        state.append(float(obstacle_x))
        state.append(float(obstacle_y))
        state.append(float(obstacle_vel_x))
        state.append(float(obstacle_vel_y))
        # Last action observation
        state.append(float(action_linear_previous))
        state.append(float(action_angular_previous))
        # self.get_logger().info(f'DTG: {dtg:.2f} ATG: {atg:.2f} X: {x:.2f} Y: {y:.2f} Θ: {theta:.2f} || Ω: {omega:.2f}  || OX: {obstacle_x:.2f} OY: {obstacle_y:.2f} OVX: {obstacle_vel_x:.2f} OVY: {obstacle_vel_y:.2f}')
        # self.get_logger().info(f'State: {state}')
        self.local_step += 1
        return state
    
    def episode_check(self):
        # self.get_logger().info(f"Obstacle distance: {self.obstacle_distance_nearest:.2f}")
        # Success
        if self.robot.distance_to_goal < REAL_THRESHOLD_GOAL:
            self.get_logger().info(bcolors.OKGREEN + "Episode done, Agent reached the goal!" + bcolors.ENDC)
            self._EP_succeed = SUCCESS
        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self.get_logger().info(bcolors.WARNING + "Episode done, Agent reached the timeout!" + bcolors.ENDC)
            self._EP_succeed = TIMEOUT
        # Collision
        elif self.obstacle_distance_nearest < REAL_THRESHOLD_COLLISION:
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

        # Environment is not ready
        self.env_ready = False

    
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
        # Initialize the Episode and response
        if len(request.action) == 0:
            self.get_logger().info(bcolors.OKGREEN + "Episode started, Agent is moving..." + bcolors.ENDC)
            response.state = self.get_state(action_linear_previous=0.0, action_angular_previous=0.0)
            response.reward = 0.0
            response.done = False
            response.success = UNKNOWN
            response.distance_traveled = 0.0
            return response

        # Get the action
        action_linear = (request.action[LINEAR_VELOCITY_LOC] + 1) / 2 * REAL_SPEED_LINEAR_MAX

        action_angular = request.action[ANGULAR_VELOCITY_LOC] * REAL_SPEED_ANGULAR_MAX

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
        reward_out, [R_DISTANCE, R_ANGLE, R_WAYPOINT, R_FONT_SCAN, R_OTHER_SCAN] = self.reward_manager.get_reward(
            status              = self._EP_succeed,
            action_linear       = request.action[LINEAR_VELOCITY_LOC], # not used
            action_angular      = request.action[ANGULAR_VELOCITY_LOC], # not used
            distance_to_goal    = self.robot.distance_to_goal / MAX_GOAL_DISTANCE, # Normalize the distance
            angle_to_goal       = self.robot.goal_angle / math.pi, # Normalize the angle
            omega               = action_angular, # not used
            scan_ranges         = self.scan_ranges # Lidar scan all normalized
        )
        # self.get_logger().info(f"R_DISTANCE: {R_DISTANCE:.2f} R_ANGLE: {R_ANGLE:.2f} R_FONT_SCAN: {R_FONT_SCAN:.2f} R_OTHER_SCAN: {R_OTHER_SCAN:.2f}")
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

        # Return the response
        return response

def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)

    drl_gazebo.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
