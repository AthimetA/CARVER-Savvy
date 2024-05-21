#!/usr/bin/env python3
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
# Authors: Tomas

from abc import ABC, abstractmethod
import numpy as np
import torch
import torch.nn.functional as torchf
                                 
from settings.constparams import ACTION_SIZE, HIDDEN_SIZE, BATCH_SIZE, BUFFER_SIZE, DISCOUNT_FACTOR, \
                                TAU, EPSILON_DECAY, EPSILON_MINIMUM, ENABLE_VISUAL, NUM_SCAN_SAMPLES, \
                                EPSILON_INITIAL, LEARNING_RATE_ACTOR, LEARNING_RATE_CRITIC

from drlutils_replaybuffer import ReplayBuffer
from drlutils_visual import DrlVisual

class BaseAgent(ABC):
    def __init__(self, device, algorithm):
        
        # Device
        self.device = device
        self.algorithm = algorithm

        # Network structure
        self.state_size         = NUM_SCAN_SAMPLES + 2 + 6 + 4 + 2
        self.action_size        = ACTION_SIZE
        self.hidden_size        = HIDDEN_SIZE
        self.input_size         = self.state_size
        # Hyperparameters
        self.batch_size         = BATCH_SIZE
        self.buffer_size        = BUFFER_SIZE
        self.discount_factor    = DISCOUNT_FACTOR
        self.learning_rate_actor= LEARNING_RATE_ACTOR
        self.learning_rate_critic= LEARNING_RATE_CRITIC
        self.tau                = TAU
        # Other parameters
        self.loss_function      = torchf.mse_loss
        self.epsilon            = EPSILON_INITIAL
        self.epsilon_decay      = EPSILON_DECAY
        self.epsilon_minimum    = EPSILON_MINIMUM

        if ENABLE_VISUAL:
            self.visual = None

        self.networks = []
        self.iteration = 0

    @abstractmethod
    def train(self, replaybuffer: ReplayBuffer):
        pass

    @abstractmethod
    def get_action():
        pass

    @abstractmethod
    def get_action_random():
        pass

    def create_network(self, type: torch.nn.Module, name: str) -> torch.nn.Module:
        network = type(name, self.input_size, self.action_size, self.hidden_size).to(self.device)
        self.networks.append(network)
        return network

    def hard_update(self, target: torch.nn.Module, source: torch.nn.Module):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(param.data)

    def soft_update(self, target: torch.nn.Module, source: torch.nn.Module, tau: float):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

    def get_model_configuration(self):
        configuration = ""
        for attribute, value in self.__dict__.items():
            if attribute not in ['actor', 'actor_target', 'critic', 'critic_target']:
                configuration += f"{attribute} = {value}\n"
        return configuration

    def get_model_parameters(self):
        parameters = [self.batch_size, self.buffer_size, self.state_size, self.action_size, self.hidden_size,
                            self.discount_factor, self.learning_rate_actor, self.learning_rate_critic, self.tau,]
        parameter_string = ', '.join(map(str, parameters))
        return parameter_string

    def attach_visual(self, visual: DrlVisual):
        if self.algorithm == 'sac':
            self.actor.visual = visual
            self.critic_1.visual = visual
        else:
            self.actor.visual = visual
            self.critic.visual = visual
        # State visual
        self.visual = visual

class OUNoise(object):
    def __init__(self, action_space, mu=0.0, theta=0.15, max_sigma=0.2, min_sigma=0.1, decay_period=600_000):
        self.mu = mu
        self.theta = theta
        self.sigma = max_sigma
        self.max_sigma = max_sigma
        self.min_sigma = min_sigma
        self.decay_period = decay_period
        self.action_dim = action_space
        self.reset()

    def reset(self):
        self.state = np.ones(self.action_dim) * self.mu

    def evolve_state(self):
        x = self.state
        dx = self.theta * (self.mu - x) + self.sigma * np.random.randn(self.action_dim)
        self.state = x + dx
        return self.state

    def get_noise(self, t=0):
        ou_state = self.evolve_state()
        decaying = float(float(t) / self.decay_period)
        self.sigma = max(self.sigma - (self.max_sigma - self.min_sigma) * min(1.0, decaying), self.min_sigma)
        return ou_state