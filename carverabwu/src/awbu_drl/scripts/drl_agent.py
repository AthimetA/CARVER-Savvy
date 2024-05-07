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

from settings.constparams import ENABLE_VISUAL, ENABLE_STACKING, OBSERVE_STEPS, MODEL_STORE_INTERVAL, GRAPH_DRAW_INTERVAL

from awbu_interfaces.srv import DrlStep, EnvReady
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

import torch

from env_utils import get_simulation_speed, read_stage, translate_outcome
from env_utils import bcolors

from drlagnet_td3 import TD3
from drlutils_graph import Graph
from drlutils_replaybuffer import ReplayBuffer
from drlutils_storagemanager import StorageManager
from drlutils_logger import Logger
from drlutils_visual import *

class DrlAgent(Node):
    def __init__(self,
    algorithm : str = "td3",
    training : bool = True,
    real_robot : bool= False,
    load_session : bool = True,
    ):
        super().__init__("DrlAgentNode")

        self.test = -1


        '''
        
        Agent parameters:
        
        '''

        self.algorithm = algorithm
        self.training = int(training)
        self.load_session = load_session
        self.real_robot = real_robot

        # ===================================================================== #
        #                             Initialization                            #
        # ===================================================================== #

        # Torch device
        if (torch.cuda.is_available()):
            self.get_logger().info(bcolors.OKGREEN + f'GPU available: {torch.cuda.get_device_name(0)}' + bcolors.ENDC)
        else:
            self.get_logger().info(bcolors.FAIL + 'GPU not available, using CPU' + bcolors.ENDC)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.stage = read_stage(stage=None)
        self.get_logger().info(bcolors.OKGREEN + f"Agent Mode: {'Training' if self.training else 'Testing'}, at Stage: {self.stage}" + bcolors.ENDC)
        self.sim_speed = get_simulation_speed(stage=self.stage) if not self.real_robot else 1
        self.get_logger().info(bcolors.OKGREEN + f"Simulation Speed: {self.sim_speed}" + bcolors.ENDC)

        self.total_steps = 0
        self.observe_steps = OBSERVE_STEPS

        # ===================================================================== #
        #                             Model loading                             #
        # ===================================================================== #

        # Initialize the model
        self.model = TD3(self.device, self.sim_speed)
        self.get_logger().info(bcolors.OKBLUE + f"Algorithm: {self.algorithm}, Model Initialized" + bcolors.ENDC)

        
        # Initialize the replay buffer
        self.replay_buffer = ReplayBuffer(self.model.buffer_size)
        self.get_logger().info(bcolors.OKBLUE + f"Replay Buffer Initialized with size: {self.model.buffer_size}" + bcolors.ENDC)

        # Initialize the graph
        self.graph = Graph()
        self.get_logger().info(bcolors.OKBLUE + "Graph Initialized" + bcolors.ENDC)

        # Initialize the storage manager
        self.sm = StorageManager(algorithm  =   self.algorithm, # Algorithm used
                                 stage      =   self.stage, # Stage number
                                 device     =   self.device # Torch device
                                 )

        # If the model directory does not exist, Load the session is False
        if not os.path.exists(self.sm.model_dir):
            self.load_session = False
        
        # If loading a session, load the model
        if self.load_session:
            # Delete the model
            del self.model
            self.model = self.sm.load_model()
            self.model.device = self.device
            self.sm.load_weights(self.model.networks)
            
            # Load the replay buffer
            if self.training:
                self.replay_buffer.buffer = self.sm.load_replay_buffer(self.model.buffer_size)
            # Load the graph data
            self.total_steps = self.graph.set_graphdata(self.sm.load_graphdata(), self.sm.episode)

            self.get_logger().info(bcolors.OKGREEN + f"Model Loaded: {self.model.get_model_parameters()} device: {self.model.device} total steps: {self.total_steps}" + bcolors.ENDC)      

        else:
            self.sm.new_session()
            self.sm.store_model(self.model)

        self.get_logger().info(bcolors.OKBLUE + "Storage Manager Initialized" + bcolors.ENDC)

        # Update graph session dir
        self.graph.session_dir = self.sm.session_dir

        # Initialize the logger
        self.logger = Logger(   training        = self.training,  
                                session         = self.sm.session,
                                stage           = self.stage,
                                load_episode    = self.sm.episode,
                                algorithm       = self.algorithm,
                                session_dir     = self.sm.session_dir, # Use for comparison file
                                state_dir       = self.sm.stage_dir, # Use for log file
                                hyperparameters = self.model.get_model_parameters(),
                                model_config    = self.model.get_model_configuration(),
                            )
        self.get_logger().info(bcolors.OKBLUE + "Logger Initialized" + bcolors.ENDC)
        
        if ENABLE_VISUAL:
            self.qapp = QtWidgets.QApplication([])
            self.visual = DrlVisual(self.model.state_size, self.model.hidden_size)
            self.model.attach_visual(self.visual)

        # ===================================================================== #
        #                             Start Process                             #
        # ===================================================================== #
        # Create Clients for step action and goal position services
        self.step_comm_client = self.create_client(DrlStep, 'step_comm')
        self.env_comm_client = self.create_client(EnvReady, 'env_comm')
        if not self.real_robot:
            self.gazebo_pause = self.create_client(Empty, '/pause_physics')
            self.gazebo_unpause = self.create_client(Empty, '/unpause_physics')
        # Start the process
        self.timer_hz = 30.0 * self.sim_speed # Scale the simulation speed
        self.timer_period = 1e9/self.timer_hz # Convert to nanoseconds
        self.episode_start_time = 0.0
        self.episode_done = False
        self.agent_status = "IDLE"

        self.episode_radom_action = False

        self.process_start_time = time.perf_counter_ns()

        self.get_logger().info(bcolors.OKGREEN + "DRL Agent Node Initialized, Ready to Start..." + bcolors.ENDC)

        self.agent_process()

    def pause_simulation(self):
        if self.real_robot:
            return None # No need to pause simulation for real robot
        # Pause simulation
        while not self.gazebo_pause.wait_for_service():
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
        while not self.gazebo_unpause.wait_for_service():
            self.get_logger().info('unpause gazebo service not available, waiting again...')
        future = self.gazebo_unpause.call_async(Empty.Request())
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                return None # Simulation unpaused

    def get_goal_status(self):
        # Request new goal position
        req = EnvReady.Request()
        # Wait for service to be available
        while not self.env_comm_client.wait_for_service():
            self.get_logger().info('new goal service not available, waiting again...')
        # Call the service
        future = self.env_comm_client.call_async(req)
        # Wait for response
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    # Get the response from the service
                    # res ---> Goal.Response()
                    res = future.result()
                    return res.env_status
                else:
                    self.get_logger().error(
                        'Exception while calling service: {0}'.format(future.exception()))
                    print("ERROR getting   service response!")

    def wait_new_goal(self):
        while(self.get_goal_status() == False):
            print(f'Goal status: {self.get_goal_status()}')
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
        while not self.step_comm_client.wait_for_service():
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

        # x % chance of random action
        if np.random.rand() < 0.5:
            self.episode_radom_action = True
        else:
            self.episode_radom_action = False

        return state
    
    def agent_process(self):
        while (True):
            
            # Get the current time
            current_time = time.perf_counter_ns()

            if (current_time - self.process_start_time) > self.timer_period:

                # self.get_logger().info(f"Difference: {current_time - self.process_start_time}")
                # self.get_logger().info(f"Agent Status: {self.agent_status}")

                if self.agent_status == "IDLE":
                    # Prepare the environment
                    self.pause_simulation()
                    # Wait for environment to be ready
                    self.wait_new_goal()
                    # Start the Episode
                    self.agent_status = "EPISODE STARTED"

                    # Initialize the episode
                    self.episode_done = False
                    step, reward_sum, loss_critic, loss_actor = 0, 0, 0, 0
                    action_past = [0.0, 0.0]
                    state = self.init_episode()

                    if ENABLE_STACKING:
                        frame_buffer = [0.0] * (self.model.state_size * self.model.stack_depth * self.model.frame_skip)
                        state = [0.0] * (self.model.state_size * (self.model.stack_depth - 1)) + list(state)
                        next_state = [0.0] * (self.model.state_size * self.model.stack_depth)

                    # Unpause the simulation
                    self.unpause_simulation()

                    # Start the episode timer
                    self.episode_start_time = time.perf_counter()

                    self.test *= -1 # Toggle the test value

                elif self.agent_status == "EPISODE STARTED":
                    
                    # # Get the current action
                    # action = self.get_random_action()

                    if self.training and self.total_steps < self.observe_steps:
                        action = self.model.get_action_random()
                    else:
                        action = self.model.get_action(state, self.training, step, ENABLE_VISUAL)
                        # if self.episode_radom_action:
                        #     action = self.model.get_action_random()
                        # else:
                        #     action = self.model.get_action(state, self.training, step, ENABLE_VISUAL)

                    # Set the current action 
                    action_current = action

                    # Take a step
                    next_state, reward, self.episode_done, outcome, distance_traveled = self.step(action_current, action_past)

                    # Update the past action
                    action_past = copy.deepcopy(action_current)
                    # Update the reward sum
                    reward_sum += reward


                    if ENABLE_STACKING:
                        frame_buffer = frame_buffer[self.model.state_size:] + list(next_state)      # Update big buffer with single step
                        next_state = []                                                         # Prepare next set of frames (state)
                        for depth in range(self.model.stack_depth):
                            start = self.model.state_size * (self.model.frame_skip - 1) + (self.model.state_size * self.model.frame_skip * depth)
                            next_state += frame_buffer[start : start + self.model.state_size]

                    # Train
                    if self.training == True:
                        self.replay_buffer.add_sample(state, action, [reward], next_state, [self.episode_done])
                        if self.replay_buffer.get_length() >= self.model.batch_size:
                            loss_c, loss_a, = self.model._train(self.replay_buffer)
                            loss_critic += loss_c
                            loss_actor += loss_a

                    if ENABLE_VISUAL:
                        self.visual.update_reward(reward_sum)
                    
                    # Update the state
                    state = copy.deepcopy(next_state)
                    step += 1

                    # Check if the episode is done
                    if self.episode_done:
                        self.agent_status = "EPISODE DONE"

                elif self.agent_status == "EPISODE DONE":
                    self.pause_simulation()
                    self.total_steps += step
                    duration = time.perf_counter() - self.episode_start_time
                    # Finish the episode
                    self.finish_episode(step, duration, outcome, distance_traveled, reward_sum, loss_critic, loss_actor)
                    self.agent_status = "IDLE"


                # Reset the process start time
                self.process_start_time = time.perf_counter_ns()
            else:
                # Wait for the next loop
                pass

    def finish_episode(self, step, eps_duration, outcome, dist_traveled, reward_sum, loss_critic, lost_actor):
            
            if self.total_steps < self.observe_steps:
                print(f"Observe phase: {self.total_steps}/{self.observe_steps} steps")
                return

            # Update the episode number
            self.sm.update_episode()
            print(f"Epi: {self.sm.episode:<5}R: {reward_sum:<8.0f}outcome: {translate_outcome(outcome):<13}", end='')
            print(f"steps: {step:<6}steps_total: {self.total_steps:<7}time: {eps_duration:<6.2f}")

            if (not self.training):
                self.logger.update_test_results(step, outcome, dist_traveled, eps_duration, 0)
                return

            # Update the graph
            self.graph.update_data(step, self.total_steps, outcome, reward_sum, loss_critic, lost_actor)
            # Update the logger file
            self.logger.file_log.write(f"{self.sm.episode}, {reward_sum}, {outcome}, {eps_duration}, {step}, {self.total_steps}, \
                                            {self.replay_buffer.get_length()}, {loss_critic / step}, {lost_actor / step}\n")

            if (self.sm.episode % MODEL_STORE_INTERVAL == 0):
                self.sm.save_session(
                networks            =   self.model.networks,
                graph_pickle_data   =   self.graph.graphdata, 
                replay_buffer       =   self.replay_buffer.buffer)
                self.logger.update_comparison_file(self.sm.episode, self.graph.get_success_count(), self.graph.get_reward_average())
            if (self.sm.episode % GRAPH_DRAW_INTERVAL == 0) or (self.sm.episode == 1):
                self.graph.draw_plots(self.sm.episode)


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
