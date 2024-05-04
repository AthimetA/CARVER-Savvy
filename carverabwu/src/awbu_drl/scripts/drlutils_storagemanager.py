from collections import deque
import os
import io
import pickle
import socket
import torch
import json

from env_utils import bcolors

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

        print(bcolors.OKGREEN + f"Algorithm: {self.algorithm}, Session: {self.session}, Stage: {self.stage}, Episode: {self.episode}" + bcolors.ENDC)

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

        print(bcolors.OKGREEN + f"Session directory: {self.session_dir}" + bcolors.ENDC)

        # Create the stage directory if it does not exist
        if not os.path.exists(self.stage_dir):
            os.makedirs(self.stage_dir)

    '''
    
    General functions
    
    '''

    def delete_file(path):
        if os.path.exists(path):
            os.remove(path)

    '''
    
    Saving functions

    
    '''

    # Store the model structure when creating the model on the first episode
    def store_model(self, model):
        model_file_name = f'{self.algorithm}_agent_model.pkl'
        dir = os.path.join(self.stage_dir, model_file_name)
        with open(dir, 'wb') as f:
            pickle.dump(model, f, pickle.HIGHEST_PROTOCOL)




class CpuUnpickler(pickle.Unpickler):
    def __init__(self, file, device):
        self.device = device
        super(CpuUnpickler, self).__init__(file)

    def find_class(self, module, name):
        if module == 'torch.storage' and name == '_load_from_bytes':
            return lambda b: torch.load(io.BytesIO(b), device=self.device)
        else:
            return super().find_class(module, name)