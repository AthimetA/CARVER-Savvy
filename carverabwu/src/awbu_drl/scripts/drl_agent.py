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

# Modified by: Athimet Aiewcharoen, FIBO, KMUTT
# Date: 2024-04-24

import copy
import os
import sys
import time
import numpy as np

from settings.constparams import ENABLE_VISUAL, ENABLE_STACKING, OBSERVE_STEPS, MODEL_STORE_INTERVAL, GRAPH_DRAW_INTERVAL, STEP_TIME


from awbu_interfaces.srv import DrlStep, Goal
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node


class DrlAgent(Node):
    def __init__(self,
    training = True,
    load_episode = 0,
    real_robot = False
    ):
        super().__init__("DrlAgentNode")
        self.training = int(training)
        self.episode = int(load_episode)
        self.real_robot = real_robot
        self.total_steps = 0
        self.observe_steps = OBSERVE_STEPS
        self.test = -1
        # ===================================================================== #
        #                             Start Process                             #
        # ===================================================================== #
        # Create Clients for step action and goal position services
        self.step_comm_client = self.create_client(DrlStep, 'step_comm')
        self.goal_comm_client = self.create_client(Goal, 'goal_comm')
        if not self.real_robot:
            self.gazebo_pause = self.create_client(Empty, '/pause_physics')
            self.gazebo_unpause = self.create_client(Empty, '/unpause_physics')
        # Start the process
        self.process()

    def pause_simulation(self):
        if self.real_robot:
            return None # No need to pause simulation for real robot
        # Pause simulation
        while not self.gazebo_pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pause gazebo service not available, waiting again...')
        future = self.gazebo_pause.call_async(Empty.Request())
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                return None # Simulation paused
            
    def unpause_simulation(self):
        if self.real_robot:
            return None # No need to unpause simulation for real robot
        # Unpause simulation
        while not self.gazebo_unpause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('unpause gazebo service not available, waiting again...')
        future = self.gazebo_unpause.call_async(Empty.Request())
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                return None # Simulation unpaused

    def get_goal_status(self):
        # Request new goal position
        req = Goal.Request()
        # Wait for service to be available
        while not self.goal_comm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('new goal service not available, waiting again...')
        # Call the service
        future = self.goal_comm_client.call_async(req)
        # Wait for response
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    # Get the response from the service
                    # res ---> Goal.Response()
                    res = future.result()
                    return res.new_goal # Return the new goal position
                else:
                    self.get_logger().error(
                        'Exception while calling service: {0}'.format(future.exception()))
                    print("ERROR getting   service response!")

    def wait_new_goal(self):
        while(self.get_goal_status() == False):
            print("Waiting for new goal... (if persists: reset gazebo_goals node)")
            time.sleep(1.0)

    def step(self,
    action: list,
    previous_action: list
    ):
        # Request to take a step
        req = DrlStep.Request()
        req.action = action
        req.previous_action = previous_action
        # Wait for service to be available
        while not self.step_comm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('env step service not available, waiting again...')
        future = self.step_comm_client.call_async(req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    res = future.result()
                    return res.state, res.reward, res.done, res.success, res.distance_traveled
                else:
                    self.get_logger().error(
                        'Exception while calling service: {0}'.format(future.exception()))
                    print("ERROR getting step service response!")

    def init_episode(self):
        state, _, _, _, _ = self.step(action=[], previous_action=[0.0, 0.0])
        return state
    
    def get_random_action(self):
        # Get a random action
        vl = np.random.uniform(0.0, 6.0)
        vw = np.random.randint(-3, 3)
        return [5.0*self.test, 0.0]

    def process(self):
        # Prepare the environment
        self.pause_simulation()

        while (True):
            # Get the goal position from service
            _ = self.wait_new_goal() # Wait for new goal to be created on DRL Gazebo Node [Vaule is not used]
            self.test *= -1
            # Initialize the episode
            episode_done = False
            step, reward_sum, loss_critic, loss_actor = 0, 0, 0, 0
            action_past = [0.0, 0.0]
            state = self.init_episode()

            self.unpause_simulation()
            # Wait for the simulation to start
            time.sleep(0.5) # Delay for 0.5 seconds
            # Start the episode timer
            episode_start = time.perf_counter()

            while not episode_done:
                self.get_logger().info(f"Step: {step}")
                # If total steps is less than observe steps, take random actions
                if self.training and self.total_steps < self.observe_steps:
                    action = self.get_random_action()
                else:
                    action = self.get_random_action()

                # Get the current action
                action_current = action

                # Take a step
                next_state, reward, episode_done, outcome, distance_traveled = self.step(action_current, action_past)
                # Update the past action
                action_past = copy.deepcopy(action_current)
                # Update the reward sum
                reward_sum += reward

                state = copy.deepcopy(next_state)
                step += 1

                time.sleep(STEP_TIME) # Delay for STEP_TIME seconds

            # Episode done
            # Pause the simulation
            self.pause_simulation()
            self.total_steps += step
            duration = time.perf_counter() - episode_start
            # Finish the episode
            self.finish_episode(step, duration, outcome, distance_traveled, reward_sum, loss_critic, loss_actor)

    def finish_episode(self, step, eps_duration, outcome, dist_traveled, reward_sum, loss_critic, lost_actor):
            if self.total_steps < self.observe_steps:
                print(f"Observe phase: {self.total_steps}/{self.observe_steps} steps")
                return None

            self.episode += 1
            print(f"Epi: {self.episode:<5}R: {reward_sum:<8.0f}", end='')
            print(f"steps: {step:<6}steps_total: {self.total_steps:<7}time: {eps_duration:<6.2f}")

def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    if len(args) == 0:
        drl_agent = DrlAgent()
    else:
        rclpy.shutdown()
        quit("ERROR: wrong number of arguments!")
    rclpy.spin(drl_agent)
    drl_agent.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
