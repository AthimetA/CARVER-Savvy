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

import copy
import os
import sys
import time
import rclpy
from rclpy.node import Node
import torch

from awbu_interfaces.srv import DrlStep, EnvReady , ScoreStep
from std_srvs.srv import Empty

from env_utils import translate_outcome,bcolors

from drlagnet_td3 import TD3
from drlagnet_sac import SAC
from drlutils_visual import *

from settings.constparams import ENABLE_VISUAL, SRV_ENV_COMM, SRV_STEP_COMM, SRV_SCORE_STEP_COMM


class DrlAgent(Node):
    def __init__(self,
    algorithm : str = "td3",
    ):
        super().__init__("DrlAgentNode")

        '''
        
        Agent parameters:
        
        '''

        self.algorithm = algorithm

        # ===================================================================== #
        #                             Initialization                            #
        # ===================================================================== #

        # Torch device
        if (torch.cuda.is_available()):
            self.get_logger().info(bcolors.OKGREEN + f'GPU available: {torch.cuda.get_device_name(0)}' + bcolors.ENDC)
        else:
            self.get_logger().info(bcolors.FAIL + 'GPU not available, using CPU' + bcolors.ENDC)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # ===================================================================== #
        #                             Model loading                             #
        # ===================================================================== #
        if self.algorithm == "sac":
            self.model = SAC(self.device, self.algorithm)
        elif self.algorithm == "td3":
            self.model = TD3(self.device, self.algorithm)
        else:
            self.get_logger().error(bcolors.FAIL + "Invalid Algorithm" + bcolors.ENDC)
            quit()

        self.get_logger().info(bcolors.OKBLUE + f"Algorithm: {self.algorithm}, Model Initialized" + bcolors.ENDC)
        
        # If the general weights is exist, load it
        __general_weights_path = os.getenv('ABWUDRL_BASE_PATH') + f"/src/awbu_drl/model/weight/{self.algorithm}"

        self.get_logger().info(bcolors.OKBLUE + f"Checking General Weights: {__general_weights_path}" + bcolors.ENDC)

        if os.path.exists(__general_weights_path):

            self.get_logger().info(bcolors.OKGREEN + f"General Weights Found" + bcolors.ENDC)

            for _network in self.model.networks:
                _network.load_state_dict(torch.load(os.path.join(__general_weights_path, f"{_network.name}.pt"), map_location=self.device))
                self.get_logger().info(bcolors.OKGREEN + f"General {_network.name} weights loaded" + bcolors.ENDC)
        else:
            self.get_logger().info(bcolors.FAIL + f"No General Weights Found" + bcolors.ENDC)
            quit()
        
        # ========================Visual Initialization======================== #
        if ENABLE_VISUAL:
            self.qapp = QtWidgets.QApplication([])
            self.visual = DrlVisual(self.model.state_size, self.model.hidden_size)
            self.model.attach_visual(self.visual)

        # ===================================================================== #
        #                             Start Process                             #
        # ===================================================================== #
        # Create Clients for step action and goal position services
        self.step_comm_client = self.create_client(DrlStep, SRV_STEP_COMM)
        self.env_comm_client = self.create_client(EnvReady, SRV_ENV_COMM)
        self.step_score_client = self.create_client(ScoreStep, SRV_SCORE_STEP_COMM)

        # Start the process
        self.timer_hz = 50.0
        self.timer_period = 1e9/self.timer_hz # Convert to nanoseconds
        self.episode_start_time = 0.0
        self.episode_done = False
        self.agent_status = "IDLE"
        self.local_ep = 0

        self.episode_radom_action = False

        self.process_start_time = time.perf_counter_ns()

        self.get_logger().info(bcolors.OKGREEN + "DRL Agent Node Initialized, Ready to Start..." + bcolors.ENDC)

        self.agent_process()

    def get_env_status(self):
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

    def get_score(self):
        req = ScoreStep.Request()
        # Wait for service to be available
        while not self.step_score_client.wait_for_service():
            self.get_logger().info('score service not available, waiting again...')
        # Call the service
        future = self.step_score_client.call_async(req)
        # Wait for response
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    res = future.result()
                    return res.k_time , res.m_time, res.total_time
                else:
                    self.get_logger().error(
                        'Exception while calling service: {0}'.format(future.exception()))
                    print("ERROR getting   service response!")
    
    def wait_env_to_be_ready(self):
        while(self.get_env_status() == False):
            self.get_logger().info(bcolors.WARNING + "Waiting for Environment to be ready... call '/abwu_drl_set_goal' service" + bcolors.ENDC)
            time.sleep(1.0)

    '''
    
    DRL Agent Step

    
    '''

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
        return state
    
    '''
    
    Test code for the agent process Hz
    
    '''

    # def agent_process(self):
    #     tick = 0

    #     process_start_time = time.perf_counter_ns()

    #     process_time_prev = 0

    #     while (True):
            
    #         # Get the current time
    #         current_time = time.perf_counter_ns()

    #         if (current_time - self.process_start_time) > self.timer_period:

    #             # self.get_logger().info(f"Difference: {current_time - self.process_start_time}")
    #             # self.get_logger().info(f"Agent Status: {self.agent_status}")  
    #             tick += 1

    #             process_time_sec = (time.perf_counter_ns() - process_start_time) / 1e9


    #             self.get_logger().info(f"Hz {1 / (process_time_sec - process_time_prev):.2f} , Avg Hz: {tick / (time.perf_counter_ns() - process_start_time) * 1e9:.2f}")

    #             process_time_prev = process_time_sec

    #             # Reset the process start time
    #             self.process_start_time = time.perf_counter_ns()
    #         else:
    #             # Wait for the next loop
    #             pass

    def agent_process(self):

        while (True):
            
            # Get the current time
            current_time = time.perf_counter_ns()

            if (current_time - self.process_start_time) > self.timer_period:
                
                if ENABLE_VISUAL:
                    # Process the visual events 
                    QtWidgets.QApplication.processEvents()

                if self.agent_status == "IDLE":
                    # Wait for environment to be ready
                    self.wait_env_to_be_ready()
                    # Start the Episode
                    self.agent_status = "EPISODE STARTED"

                    # Initialize the episode
                    self.episode_done = False
                    step, reward_sum, loss_critic, loss_actor = 0, 0, 0, 0
                    action_past = [0.0, 0.0]
                    # Initialize the state
                    state, _, _, _, _ = self.step(action=[], previous_action=[0.0, 0.0])

                    # Start the episode timer
                    self.episode_start_time = time.perf_counter()

                elif self.agent_status == "EPISODE STARTED":
                    # Get Action

                    action = self.model.get_action_real(
                        state=state, # Pass the current state just in case the model needs it
                        visualize=ENABLE_VISUAL,
                    )

                    # Set the current action 
                    action_current = action

                    # Take a step
                    next_state, reward, self.episode_done, outcome, distance_traveled = self.step(action_current, action_past)

                    # Update the past action
                    action_past = copy.deepcopy(action_current)
                    # Update the reward sum
                    reward_sum += reward

                    if ENABLE_VISUAL:
                        self.visual.update_reward(reward_sum)
                    
                    # Update the state
                    state = copy.deepcopy(next_state)
                    step += 1

                    # Check if the episode is done
                    if self.episode_done:
                        self.agent_status = "EPISODE DONE"

                elif self.agent_status == "EPISODE DONE":
                    # Calculate the episode duration
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
            k_time , m_time, total_time= self.get_score()
            __text = f"k_step : {k_time} , m_time : {m_time} , total_time : {total_time} "


            self.get_logger().info(bcolors.OKBLUE + __text + bcolors.ENDC)
            
            # Display the episode information
            __text = f"Reward: {reward_sum:<8.0f} Outcome: {translate_outcome(outcome):<13} Steps: {step:<6} Time: {eps_duration:<6.2f} HZ: {step / eps_duration:.2f} Distance: {dist_traveled:.2f}"
            self.get_logger().info(bcolors.OKGREEN + __text + bcolors.ENDC)


def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    if len(args) == 1:
        algorithm = args[0]
    else:
        algorithm = "td3"

    # Check if the algorithm is valid
    if algorithm not in ["td3", "sac"]:
        print("Invalid Algorithm Using TD3")
        algorithm = "td3"
    
    drl_agent = DrlAgent(algorithm=algorithm)

    rclpy.spin(drl_agent)
    drl_agent.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
