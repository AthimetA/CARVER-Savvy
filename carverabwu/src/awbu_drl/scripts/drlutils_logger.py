import numpy as np
from settings.constparams import COLLISION, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM
import time
import os

class Logger():
    def __init__(self,
    training: bool, # Use for file naming
    session: int, # Number of session
    stage: int, # Stage number
    load_episode: int, # Episode number
    algorithm: str, # Algorithm used
    session_dir: str,  # Use for comparison file
    state_dir: str, # Use for log file
    hyperparameters: str, # Hyperparameters used
    model_config : str, # Model configuration

    ):  
        # Initialize variables
        self.is_training = training
        self.session = str(session)
        self.stage = str(stage)
        self.load_episode = load_episode
        self.algorithm = algorithm
        self.session_dir = session_dir
        self.state_dir = state_dir

        self.file_log_dir = os.path.join(self.state_dir, "logs")

        if not os.path.exists(self.file_log_dir):
            os.makedirs(self.file_log_dir)


        self.hyperparameters = hyperparameters
        self.model_config = model_config


        # Initialize variables for testing
        self.test_entry = 0
        self.test_outcome = [0] * RESULTS_NUM
        self.test_distance = []
        self.test_duration = []
        self.test_swerving = []

        # Initialize variables for comparison
        self.highest_reward = -np.inf
        self.best_episode_reward = 0
        self.highest_success = 0
        self.best_episode_success = 0

        # Initialize files
        datetime = time.strftime("%Y%m%d-%H%M%S")

        # Folder for file_log at datetime
        self.file_log_datetime_dir = os.path.join(self.file_log_dir, f"{self.algorithm}_{datetime}")
        if not os.path.exists(self.file_log_datetime_dir):
            os.makedirs(self.file_log_datetime_dir)
        

        self.file_comparison = self.init_comparison_file(datetime, self.session_dir, self.stage, self.hyperparameters, self.algorithm, self.session, self.load_episode)
        
        if self.is_training:
            self.file_log = self.init_training_log(datetime, self.file_log_datetime_dir, self.stage, self.model_config)
        else:
            self.file_log = self.init_testing_log(datetime, self.file_log_datetime_dir, self.stage, self.load_episode)

    '''
    
    Comparison file Functions
    
    
    '''

    def init_comparison_file(self, datetime, path, stage, hyperparameters, algorithm, session, episode):
        prefix = "_training" if self.is_training else "_testing"
        with open(os.path.join(path, "__" + algorithm + prefix + "_comparison.txt"), 'a+') as file_comparison:
            file_comparison.write(datetime + ', ' + session + ', ' + str(episode) + ', ' + stage + ', ' + hyperparameters + '\n')
        return file_comparison

    def update_comparison_file(self, episode, success_count, average_reward=0):
        if average_reward > self.highest_reward and episode != 1:
            self.highest_reward = average_reward
            self.best_episode_reward = episode
        if success_count > self.highest_success and episode != 1:
            self.highest_success = success_count
            self.best_episode_success = episode
        datetime = time.strftime("%Y%m%d-%H%M%S")
        with open(self.file_comparison.name, 'a+') as file_comparison:
            file_comparison.seek(0)
            lines = file_comparison.readlines()
            file_comparison.seek(0)
            file_comparison.truncate()
            file_comparison.writelines(lines[:-1])
            file_comparison.write(datetime + ', ' + self.session + ', ' + self.stage + ', ' + self.hyperparameters)
            if self.is_training:
                file_comparison.write(', results, ' + str(episode) + ', ' + str(self.best_episode_success) + ': ' + str(self.highest_success) + '%, ' + str(self.best_episode_reward) + ': ' + str(self.highest_reward) + '\n')
            else:
                file_comparison.write(', results, ' + str(episode) + ', ' + str(self.best_episode_success) + ', ' + str(self.highest_success) + '%\n')

    '''
    
    Training and Testing Log Functions
    
    '''

    def init_training_log(self, datetime, path, stage, model_config):
        file_log = open(os.path.join(path, "_train_stage" + stage + "_" + datetime + '.txt'), 'w+')
        file_log.write("episode, reward, success, duration, steps, total_steps, memory length, avg_critic_loss, avg_actor_loss\n")
        with open(os.path.join(path, '_model_configuration_' + datetime + '.txt'), 'w+') as file_model_config:
            file_model_config.write(model_config + '\n')
        return file_log

    def init_testing_log(self, datetime, path, stage, load_episode):
        file_log = open(os.path.join(path, "_test_stage" + stage + "_eps" + str(load_episode) + "_" + datetime + '.txt'), 'w+')
        file_log.write(f"episode, outcome, step, episode_duration, distance, s/cw/co/t\n")
        return file_log


    def update_test_results(self, step, outcome, distance_traveled, episode_duration, swerving_sum):
        self.test_entry += 1
        self.test_outcome[outcome] += 1
        if outcome == SUCCESS:
            self.test_distance.append(distance_traveled)
            self.test_duration.append(episode_duration)
            self.test_swerving.append(swerving_sum/step)
        success_count = self.test_outcome[SUCCESS]

        self.file_log.write(f"{self.test_entry}, {outcome}, {step}, {episode_duration}, {distance_traveled}, {self.test_outcome[SUCCESS]}/{self.test_outcome[COLLISION]}/{self.test_outcome[TIMEOUT]}/{self.test_outcome[TUMBLE]}\n")
        if self.test_entry > 0 and self.test_entry % 100 == 0:
            self.update_comparison_file(self.test_entry, self.test_outcome[SUCCESS] / (self.test_entry / 100), 0)
            self.file_log.write(f"Successes: {self.test_outcome[SUCCESS]} ({self.test_outcome[SUCCESS]/self.test_entry:.2%}), "
            f"collision: {self.test_outcome[COLLISION]} ({self.test_outcome[COLLISION]/self.test_entry:.2%}), "
            f"timeouts: {self.test_outcome[TIMEOUT]}, ({self.test_outcome[TIMEOUT]/self.test_entry:.2%}), "
            f"tumbles: {self.test_outcome[TUMBLE]}, ({self.test_outcome[TUMBLE]/self.test_entry:.2%}), ")
            if success_count > 0:
                self.file_log.write(f"distance: {sum(self.test_distance)/success_count:.3f}, "
                                    f"swerving: {sum(self.test_swerving)/success_count:.3f}, "
                                    f"duration: {sum(self.test_duration)/success_count:.3f}\n")
        if self.test_entry > 0:
            print(f"Successes: {self.test_outcome[SUCCESS]} ({self.test_outcome[SUCCESS]/self.test_entry:.2%}), "
            f"collision: {self.test_outcome[COLLISION]} ({self.test_outcome[COLLISION]/self.test_entry:.2%}), "
            f"timeouts: {self.test_outcome[TIMEOUT]}, ({self.test_outcome[TIMEOUT]/self.test_entry:.2%}), "
            f"tumbles: {self.test_outcome[TUMBLE]}, ({self.test_outcome[TUMBLE]/self.test_entry:.2%}), ")
            if success_count > 0:
                print(f"distance: {sum(self.test_distance)/success_count:.3f}, "
                      f"swerving: {sum(self.test_swerving)/success_count:.3f}, "
                      f"duration: {sum(self.test_duration)/success_count:.3f}")

