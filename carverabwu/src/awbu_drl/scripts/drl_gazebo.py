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
import numpy
import time

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from awbu_interfaces.srv import RingGoal
import xml.etree.ElementTree as ET
from settings.constparams import ENABLE_TRUE_RANDOM_GOALS, ARENA_LENGTH, ARENA_WIDTH, ENABLE_DYNAMIC_GOALS

NO_GOAL_SPAWN_MARGIN = 0.3 # meters away from any wall

class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        """************************************************************
        ** Initialise variables
        ************************************************************"""

        self._entity_dir_path = os.environ['SIM_MODEL_PATH'] + '/goal_box'
        # self.get_logger().info("Goal entity directory path: " + self._entity_dir_path)
        self._entity_path = os.path.join(self._entity_dir_path, 'model.sdf')
        self.entity = open(self._entity_path, 'r').read()
        self.entity_name = 'goal_box'

        self.get_logger().info("DRL Gazebo node has been started.")
        self.get_logger().info("Goal entity path: " + self._entity_path)
        # self.get_logger().info("Goal entity Object: " + self.entity)

        # with open('/tmp/drlnav_current_stage.txt', 'r') as f:
        #     self.stage = int(f.read())
        # print(f"running on stage: {self.stage}, dynamic goals enabled: {ENABLE_DYNAMIC_GOALS}")

        self.prev_x, self.prev_y = -1, -1
        self.goal_x, self.goal_y = 0.0, 0.0

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        # Initialise publishers
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', QoSProfile(depth=10))

        # Initialise client
        self.delete_entity_client       = self.create_client(DeleteEntity, 'delete_entity')
        self.spawn_entity_client        = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client    = self.create_client(Empty, 'reset_world')
        self.gazebo_pause = self.create_client(Empty, '/pause_physics')
        self.gazebo_unpause = self.create_client(Empty, '/unpause_physics')

        # Initialise servers
        self.task_succeed_server    = self.create_service(RingGoal, 'task_succeed', self.task_succeed_callback)
        self.task_fail_server       = self.create_service(RingGoal, 'task_fail', self.task_fail_callback)

        self.obstacle_coordinates   = self.get_obstacle_coordinates()
        self.get_logger().info(f"Obstacle coordinates: {self.obstacle_coordinates}")
        self.init_drl()

        # self.timer = self.create_timer(3.0, self.test_callback)

    def test_callback(self):
        # self.get_logger().info("====================fail: task_fail_callback============================")
        # self.pause_simulation()
        # time.sleep(0.1)
        # self.delete_entity()
        # time.sleep(0.25)
        # self.reset_simulation()
        # time.sleep(0.1)
        # self.generate_goal_pose(robot_x=0.0, robot_y=0.0, radius=5.0)
        # self.publish_callback()
        # time.sleep(0.1)
        # self.unpause_simulation()
        # time.sleep(0.1)
        # self.get_logger().info(f"fail: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        self.get_logger().info("Test callback")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
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
    
    Service functions
    
    '''

    def task_succeed_callback(self, request: RingGoal.Request, response: RingGoal.Response)->RingGoal.Response:
        self.get_logger().info("====================success: task_succeed_callback============================")
        self.delete_entity()
        self.generate_goal_pose(robot_x=request.robot_pose_x, robot_y=request.robot_pose_y, radius=request.radius)
        self.publish_callback()
        time.sleep(0.1)
        self.unpause_simulation()
        self.get_logger().info(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def task_fail_callback(self, request: RingGoal.Request, response: RingGoal.Response)->RingGoal.Response:
        self.get_logger().info("====================fail: task_fail_callback============================")
        self.delete_entity()
        self.reset_simulation()
        self.generate_goal_pose(robot_x=request.robot_pose_x, robot_y=request.robot_pose_y, radius=request.radius)
        self.publish_callback()

        self.unpause_simulation()
        self.get_logger().info(f"fail: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response


    '''

    DRL Genral functions

    '''

    def init_drl(self)->None:
        self.delete_entity()
        self.reset_simulation()
        self.unpause_simulation()
        self.publish_callback()
        # time.sleep(0.1)
        self.get_logger().info(f"DRL Gazebo node has been initialized, Goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")

    def publish_callback(self)->None:
        # Publish goal pose
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        self.goal_pose_pub.publish(goal_pose)
        self.get_logger().info(f"Goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        self.spawn_entity()
        self.get_logger().info("Goal published by publishe_callback")

    def goal_is_valid(self, goal_x: float, goal_y: float)->bool:
        if goal_x > ARENA_LENGTH/2 or goal_x < -ARENA_LENGTH/2 or goal_y > ARENA_WIDTH/2 or goal_y < -ARENA_WIDTH/2:
            return False
        for obstacle in self.obstacle_coordinates:
            # Obstacle is defined by 4 points [top_right, bottom_right, bottom_left, top_left] with [x, y] coordinates
            if goal_x < obstacle[0][0] and goal_x > obstacle[2][0] and goal_y < obstacle[0][1] and goal_y > obstacle[2][1]: # check if goal is inside the obstacle
                    return False
        return True
    
    '''
    
    Goal generation functions
    
    '''
    def generate_goal_pose(self, robot_x: float, robot_y: float, radius: float)->None:
        MAX_ITERATIONS = 100
        GOAL_SEPARATION_DISTANCE = 5.0
        DYNAMIC_GOAL_RADIUS = float(radius) if radius > GOAL_SEPARATION_DISTANCE else GOAL_SEPARATION_DISTANCE
        PREDEFINED_GOAL_LOCATIONS = [[-(ARENA_LENGTH/2 - 1), -(ARENA_WIDTH/2 - 1)], [ARENA_LENGTH/2 - 1, ARENA_WIDTH/2 - 1], [ARENA_LENGTH/2 - 1, -(ARENA_WIDTH/2 - 1)], [-(ARENA_LENGTH/2 - 1), ARENA_WIDTH/2 - 1]]
        self.prev_y = self.goal_y
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
            if self.goal_is_valid(goal_x, goal_y) and math.sqrt((goal_x - self.prev_x)**2 + (goal_y - self.prev_y)**2) > GOAL_SEPARATION_DISTANCE:
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
    

    '''
    
    Obstacle functions

    '''

    def get_obstacle_coordinates(self):
        obstacle_name = ['wall_outler', 'pillar1', 'pillar2']
        obstacle_coordinates = []
        for name in obstacle_name:
            obstacle_coordinates += self._sdf_obstacle_reader(name)
        return obstacle_coordinates
    
    def _sdf_obstacle_reader(self, name: str)->list:
        path = os.environ['SIM_MODEL_PATH'] + name + '/model.sdf'
        tree = ET.parse(path)
        root = tree.getroot()
        obstacle_coordinates = []
        # Get the coordinates of the walls
        for wall in root.find('model').findall('link'):
            pose = wall.find('pose').text.split(" ")
            size = wall.find('collision').find('geometry').find('box').find('size').text.split()
            pose_x = float(pose[0])
            pose_y = float(pose[1])
            # Check if the wall is rotated
            # If the wall is rotated the size is swapped for x and y
            rotation = float(pose[-1])
            if rotation == 0 or rotation == 3.14159:
                size_x = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
            else:
                size_x = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
            # Calculate the corners of the obstacle
            step_x = size_x / 2
            step_y = size_y / 2
            top_left = [pose_x - step_x, pose_y + step_y]
            top_right = [pose_x + step_x, pose_y + step_y]
            bottom_right = [pose_x + step_x, pose_y - step_y]
            bottom_left = [pose_x - step_x, pose_y - step_y]
            # Create a list of the corners
            wall_points = [top_right, bottom_right, bottom_left, top_left]
            obstacle_coordinates.append(wall_points)
        return obstacle_coordinates

def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)

    drl_gazebo.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
