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

from collections import deque
import os
import io
import pickle
import socket
import torch
import json

from env_utils import bcolors
from settings.constparams import MODEL_STORE_INTERVAL

NUMBER_OF_MODELS_TO_STORE = 2

class StorageManager:
    def __init__(self,
    algorithm,
    stage,
    device,
    new_session = False,
    ):
        self.algorithm = algorithm
        self.stage = stage
        self.device = device
        # Get the machine directory
        self.machine_dir = (os.getenv('ABWUDRL_BASE_PATH') + '/src/awbu_drl/model/' + str(socket.gethostname()))
        # Algorithm dir
        self.algorithm_dir = os.path.join(self.machine_dir, self.algorithm)

        # Create the algorithm directory if it does not exist
        if not os.path.exists(self.algorithm_dir):
            os.makedirs(self.algorithm_dir)

        # Create the metadata using json
        self.metadata_path = os.path.join(self.algorithm_dir, "metadata.json")
        
        if os.path.exists(self.metadata_path):
            with open(self.metadata_path, 'r') as f:
                self.metadata = json.load(f)

            self.session = self.metadata['session']
            self.info = self.metadata['info']
            
            # Check if the stage is already in the metadata
            if any(d['stage'] == self.stage for d in self.metadata['info']):
                # Get the episode number for the stage
                self.episode = next(item for item in self.metadata['info'] if item['stage'] == self.stage)['episode']
            else:
                # Add the stage to the metadata
                self.metadata['info'].append({'stage': self.stage, 'episode': 0})
                self.episode = 0
                with open(self.metadata_path, 'w') as f:
                    json.dump(self.metadata, f)

        else:
            self.session = 0
            self.episode = 0
            self.metadata = {
                'algorithm': self.algorithm,
                'session': self.session,
                'info' : [
                    # List of dictionaries with the stage and episode number
                    {
                        'stage': self.stage,
                        'episode': 0,
                    }
                ]
            }
            with open(self.metadata_path, 'w') as f:
                json.dump(self.metadata, f)

        if not new_session:
            self.session_dir = os.path.join(self.algorithm_dir, f'{self.algorithm}_{self.session}')
            if not os.path.exists(self.session_dir):
                os.makedirs(self.session_dir)

        else:
            # Create a new session with 0 episodes
            self.session += 1
            self.episode = 0

            self.session_dir = os.path.join(self.algorithm_dir, f'{self.algorithm}_{self.session}')
            os.makedirs(self.session_dir)

            self.metadata['session'] = self.session
            self.metadata['episode'] = self.episode
            
        # Stage dir
        self.stage_dir = os.path.join(self.session_dir, f'stage_{self.stage}')

        # Create the stage directory if it does not exist
        if not os.path.exists(self.stage_dir):
            os.makedirs(self.stage_dir)

        print(bcolors.OKCYAN + f"Algorithm: {self.algorithm}, Session: {self.session}, Stage: {self.stage}, Episode: {self.episode}" + bcolors.ENDC)

        # Model directory
        model_file_name = f'{self.algorithm}_agent_model.pkl'
        self.model_dir = os.path.join(self.session_dir, model_file_name)

        self.episode_to_delete = 1
        self.delete_episode_flag = False

    '''
    
    General functions
    
    '''

    def delete_file(self,path):
        if os.path.exists(path):
            os.remove(path)

    def update_episode(self, save = False):
        self.episode += 1

        if save:
            if self.episode % MODEL_STORE_INTERVAL == 0:
                # Update the episode number in the metadata
                for item in self.metadata['info']:
                    if item['stage'] == self.stage:
                        item['episode'] = self.episode
                        break

                with open(self.metadata_path, 'w') as f:
                    json.dump(self.metadata, f)

    def new_session(self):

        # Check if the session directory exists and episode is not 0
        if os.path.exists(self.session_dir) and self.episode != 0:

            # Create a new session with 0 episodes

            self.session += 1
            self.episode = 0

            self.session_dir = os.path.join(self.algorithm_dir, f'{self.algorithm}_{self.session}')
            os.makedirs(self.session_dir)

            self.metadata['session'] = self.session
            self.metadata["info"] = [   
                    {
                        'stage': self.stage,
                        'episode': 0,
                    }]

            with open(self.metadata_path, 'w') as f:
                json.dump(self.metadata, f)

            # Stage dir
            self.stage_dir = os.path.join(self.session_dir, f'stage_{self.stage}')

            # Create the stage directory if it does not exist
            if not os.path.exists(self.stage_dir):
                os.makedirs(self.stage_dir)

            print(bcolors.OKBLUE + f"Algorithm: {self.algorithm}, Session: {self.session}, Stage: {self.stage}, Episode: {self.episode}" + bcolors.ENDC)

            # Model directory
            model_file_name = f'{self.algorithm}_agent_model.pkl'
            self.model_dir = os.path.join(self.session_dir, model_file_name)

            self.episode_to_delete = 1
            self.delete_episode_flag = False

    '''
    
    Saving functions

    
    '''

    # Store the model structure when creating the model on the first episode
    def store_model(self, model):
        with open(self.model_dir, 'wb') as f:
            pickle.dump(model, f, pickle.HIGHEST_PROTOCOL)

    # Store the model weights
    def network_save_weights(self, network, model_dir, stage, episode):
        
        # Save the model weights
        network_filename = f'{network.name}_stage_{stage}_episode_{episode}.pt'
        network_filepath = os.path.join(model_dir, network_filename)

        print(f"saving {network.name} model weights")

        # Save using torch.save
        torch.save(network.state_dict(), network_filepath)

    def save_session(self, networks, graph_pickle_data, replay_buffer):

        episode = self.episode
        print(bcolors.OKCYAN + f"Saving data for episode: {episode}, location: {self.stage_dir}" + bcolors.ENDC)

        # Save the model weights for actor and critic networks
        for network in networks:
            self.network_save_weights(network, self.stage_dir, self.stage, episode)

        # Store graph data
        graph_file_name = f'graph_data_stage_{self.stage}_episode_{episode}.pkl'
        with open(os.path.join(self.stage_dir, graph_file_name), 'wb') as f:
            pickle.dump(graph_pickle_data, f, pickle.HIGHEST_PROTOCOL)

        # Store latest buffer (can become very large, multiple gigabytes)
        last_buffer_file_name = f'replay_buffer_stage_{self.stage}.pkl'
        with open(os.path.join(self.stage_dir, last_buffer_file_name), 'wb') as f:
            pickle.dump(replay_buffer, f, pickle.HIGHEST_PROTOCOL)

        # Delete previous iterations (Restore only the last n models)
        if self.episode > NUMBER_OF_MODELS_TO_STORE * MODEL_STORE_INTERVAL:
            self.delete_episode_flag = True
            self.episode_to_delete = self.episode - NUMBER_OF_MODELS_TO_STORE * MODEL_STORE_INTERVAL

        if self.delete_episode_flag:
            # Delete the model weights
            for network in networks:
                network_filename = f'{network.name}_stage_{self.stage}_episode_{self.episode_to_delete}.pt'
                network_filepath = os.path.join(self.stage_dir, network_filename)
                self.delete_file(network_filepath)

            # Delete the graph data
            graph_file_name = f'graph_data_stage_{self.stage}_episode_{self.episode_to_delete}.pkl'
            graph_path = os.path.join(self.stage_dir, graph_file_name)
            self.delete_file(graph_path)

            print(bcolors.FAIL + f"Deleting episode: {self.episode_to_delete}" + bcolors.ENDC)
        
    '''
    
    Loading functions
    
    
    '''
    def load_model(self):
        try :
            with open(self.model_dir, 'rb') as f:
                return CpuUnpickler(f, self.device).load()
        except FileNotFoundError:
            quit(f"The specified model: {self.model_dir} was not found. Check whether you specified the correct stage {self.stage} and model name")

    def network_load_weights(self, network, model_dir, stage):
        filepath = os.path.join(model_dir, f'{network.name}_stage_{stage}_episode_{self.episode}.pt')
        print(f"loading: {network.name} model from file: {filepath}")
        network.load_state_dict(torch.load(filepath, self.device))

    def load_weights(self, networks):
        for network in networks:
            self.network_load_weights(network, self.stage_dir, self.stage)

    def load_graphdata(self):
        graph_file_name = f'graph_data_stage_{self.stage}_episode_{self.episode}.pkl'
        graph_path = os.path.join(self.stage_dir, graph_file_name)
        if os.path.exists(graph_path):
            with open(graph_path, 'rb') as f:
                return pickle.load(f)
        else:
            print(f"graph data does not exist: {graph_path}")
            return None
        
    def load_replay_buffer(self, size):
        last_buffer_file_name = f'replay_buffer_stage_{self.stage}.pkl'
        buffer_path = os.path.join(self.stage_dir, last_buffer_file_name)
        if (os.path.exists(buffer_path)):
            with open(buffer_path, 'rb') as f:
                return pickle.load(f)
        else:
            print(f"buffer does not exist: {buffer_path}")
            return deque(maxlen=size)

class CpuUnpickler(pickle.Unpickler):
    def __init__(self, file, map_location):
        self.map_location = map_location
        super(CpuUnpickler, self).__init__(file)

    def find_class(self, module, name):
        if module == 'torch.storage' and name == '_load_from_bytes':
            return lambda b: torch.load(io.BytesIO(b), map_location=self.map_location)
        else:
            return super().find_class(module, name)