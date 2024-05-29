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

from env_utils import get_simulation_speed, read_stage, translate_outcome
from env_utils import bcolors

from drlagnet_td3 import TD3
from drlagnet_sac import SAC
from drlutils_graph import Graph
from drlutils_test_graph import Test_Graph
from drlutils_replaybuffer import ReplayBuffer
from drlutils_storagemanager import StorageManager
from drlutils_logger import Logger
from drlutils_visual import *

from settings.constparams import ENABLE_VISUAL, OBSERVE_STEPS, MODEL_STORE_INTERVAL, GRAPH_DRAW_INTERVAL


class DrlAgent(Node):
    def __init__(self,
    algorithm : str = "td3",
    training : bool = True,
    real_robot : bool= False,
    load_session : bool = True,
    ):
        super().__init__("DrlAgentNode")

        '''
        
        Agent parameters:
        
        '''

        self.algorithm = algorithm
        self.training = training
        self.EPISODE_TEST = 1000

        if self.training : self.get_logger().info(bcolors.OKGREEN + f'Start Trainning {self.algorithm}')
        else : self.get_logger().info(bcolors.OKGREEN + f'Start Evaluate {self.algorithm} with {self.EPISODE_TEST} episode')
        
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
        # self.algorithm = 'sac'
        # Initialize the model
        if self.algorithm == "sac":
            self.model = SAC(self.device, self.algorithm)
            self.observe_steps = 0 # SAC does not need observe steps
        elif self.algorithm == "td3":
            self.model = TD3(self.device, self.algorithm)
        else:
            self.get_logger().error(bcolors.FAIL + "Invalid Algorithm" + bcolors.ENDC)
            quit()
        self.get_logger().info(bcolors.OKBLUE + f"Algorithm: {self.algorithm}, Model Initialized" + bcolors.ENDC)

        
        # Initialize the replay buffer
        self.replay_buffer = ReplayBuffer(self.model.buffer_size)
        self.get_logger().info(bcolors.OKBLUE + f"Replay Buffer Initialized with size: {self.model.buffer_size}" + bcolors.ENDC)

        # Initialize the storage manager
        self.sm = StorageManager(algorithm  =   self.algorithm, # Algorithm used
                                 stage      =   self.stage, # Stage number
                                 device     =   self.device # Torch device
                                 )

        # If the model directory does not exist, Load the session is False
        if not os.path.exists(self.sm.model_dir):
            self.load_session = False
        else:
            # Session directory exists 
            if self.sm.episode == 0:
                # New session
                self.get_logger().info(bcolors.FAIL + "Session directory exists, but no episode found" + bcolors.ENDC)
                self.get_logger().info(bcolors.FAIL + "Starting a new session" + bcolors.ENDC)
                self.load_session = False
        
        # If loading a session, load the model
        if self.load_session:
            # Set the model device
            self.model.device = self.device
            # Load Network weights
            self.sm.load_weights(self.model.networks)
            
            # Load the replay buffer
            if self.training:
                self.replay_buffer.buffer = self.sm.load_replay_buffer(self.model.buffer_size)

            self.get_logger().info(bcolors.OKGREEN + f"Model Loaded: {self.model.get_model_parameters()} device: {self.model.device} total steps: {self.total_steps}" + bcolors.ENDC)      

        else:
            self.sm.new_session()
            self.sm.store_model(self.model)

            # If the general weights is exist, load it
            __general_weights_path = os.getenv('ABWUDRL_BASE_PATH') + f"/src/awbu_drl/model/weight/{self.algorithm}"

            self.get_logger().info(bcolors.OKBLUE + f"Checking General Weights: {__general_weights_path}" + bcolors.ENDC)

            if os.path.exists(__general_weights_path):

                self.get_logger().info(bcolors.OKGREEN + f"General Weights Found" + bcolors.ENDC)

                for _network in self.model.networks:
                    _network.load_state_dict(torch.load(os.path.join(__general_weights_path, f"{_network.name}.pt")))
                    self.get_logger().info(bcolors.OKGREEN + f"General {_network.name} weights loaded" + bcolors.ENDC)

                self.observe_steps = 0 # Not need observe steps

        self.get_logger().info(bcolors.OKBLUE + "Storage Manager Initialized" + bcolors.ENDC)

        if self.training :
            # Initialize the graph
            self.graph = Graph(session_dir=self.sm.session_dir, first_episode=self.sm.episode, continue_graph=False)
            # Load the graph data
            if self.load_session:
                self.total_steps = self.graph.set_graphdata(self.sm.load_graphdata(), self.sm.episode)
                self.graph.draw_plots(self.sm.episode)
            self.get_logger().info(bcolors.OKBLUE + "Graph Initialized" + bcolors.ENDC)

            # Update graph session dir
            self.graph.session_dir = self.sm.session_dir
        else : 
            self.test_graph = Test_Graph(session_dir=self.sm.session_dir, first_episode=self.sm.episode, continue_graph=True)

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
        self.step_score_client = self.create_client(ScoreStep, 'score_step_comm')

        if not self.real_robot:
            self.gazebo_pause = self.create_client(Empty, '/pause_physics')
            self.gazebo_unpause = self.create_client(Empty, '/unpause_physics')
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

                    # Unpause the simulation
                    self.unpause_simulation()

                    # Start the episode timer
                    self.episode_start_time = time.perf_counter()

                elif self.agent_status == "EPISODE STARTED":
                    # Get Action
                    if self.training and self.total_steps < self.observe_steps:
                        action = self.model.get_action_random(
                            state=state, # Pass the current state just in case the model needs it
                        )
                    else:
                        '''
                        
                        Normal action selection
                        
                        '''
                        action = self.model.get_action(
                            state=state,
                            is_training=self.training,
                            step=self.total_steps,
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

                    # Train
                    if self.training == True:
                        # Add the sample to the replay buffer
                        self.replay_buffer.add_sample(state, action, [reward], next_state, [self.episode_done])

                        # Train the model if the replay buffer is more than the batch size
                        if self.replay_buffer.get_length() >= self.model.batch_size:
                            loss_c, loss_a, = self.model.train(self.replay_buffer)
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
                        self.pause_simulation()

                elif self.agent_status == "EPISODE DONE":
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
            k_time , m_time, total_time= self.get_score()
            __text = f"k_step : {k_time} , m_time : {m_time} , total_time : {total_time} "


            self.get_logger().info(bcolors.OKBLUE + __text + bcolors.ENDC)

            if self.total_steps < self.observe_steps and self.training:
                print(f"Observe phase: {self.total_steps}/{self.observe_steps} steps")
                return

            # Update the episode number
            if self.training: # Training
                self.sm.update_episode(save = True)

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
                    self.logger.update_comparison_file(self.sm.episode, self.graph.get_success_count(), self.graph.get_reward_average(), avg_ego=0, avg_social=0)
                if (self.sm.episode % GRAPH_DRAW_INTERVAL == 0) or (self.sm.episode == 1):
                    self.graph.draw_plots(self.sm.episode)

            else: # Testing
                self.sm.update_episode(save = False)
                self.logger.update_test_results(step, 
                                                outcome, 
                                                dist_traveled, 
                                                eps_duration, 
                                                0,
                                                k_time,
                                                m_time,
                                                total_time,
                                                )
                self.local_ep  +=1 
                self.test_graph.update_data(step, self.total_steps, outcome, k_time, m_time, total_time)
                if (self.local_ep  % self.EPISODE_TEST == 0):
                    self.test_graph.draw_plots(self.local_ep, save=True)
                    self.get_logger().info(bcolors.OKGREEN + f"Test Graph Drawn at Episode: {self.local_ep}" + bcolors.ENDC)
                    # Terminate the process
                    quit()
                elif (self.local_ep % GRAPH_DRAW_INTERVAL == 0) or (self.local_ep == 1):
                    self.test_graph.draw_plots(self.local_ep, save=False)
            
            # Display the episode information
            __text = f"Episode: {self.sm.episode:<5} Reward: {reward_sum:<8.0f}Outcome: {translate_outcome(outcome):<13}Steps: {step:<6}Total Steps: {self.total_steps:<7}Time: {eps_duration:<6.2f} HZ: {step / eps_duration:.2f}"
            self.get_logger().info(bcolors.OKGREEN + __text + bcolors.ENDC)


def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    if len(args) == 1:
        algorithm = args[0]
        process = "train"

    elif len(args) == 2:
        algorithm = args[0]
        process = args[1]

    else:
        algorithm = "td3"
        process = "train"

    # Check if the algorithm is valid
    if algorithm not in ["td3", "sac"]:
        print("Invalid Algorithm Using TD3")
        algorithm = "td3"

    if process.lower() == "test":
        train = False
    else  : train = True

    
    drl_agent = DrlAgent(algorithm=algorithm, training=train, real_robot=False, load_session=True)

    rclpy.spin(drl_agent)
    drl_agent.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
