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

# Implementation of Soft Actor-Critic
# Paper: https://arxiv.org/abs/1801.01290
# Original implementation: https://github.com/haarnoja/sac [Stable Baselines3 implementation]

# Modified by:  Athimet Aiewcharoen     , FIBO, KMUTT
#               Tanut   Bumrungvongsiri , FIBO, KMUTT
# Date : 2024-05-26

import numpy as np
import copy

import torch
import torch.nn.functional as F
import torch.nn as nn
from torch.optim.lr_scheduler import StepLR, ReduceLROnPlateau

from settings.constparams import POLICY_NOISE, POLICY_NOISE_CLIP, POLICY_UPDATE_FREQUENCY

from drlagent_base_agent import BaseAgent, OUNoise

from drlutils_visual import DrlVisual

from drlutils_replaybuffer import ReplayBuffer

from abc import ABC, abstractmethod

LINEAR = 0
ANGULAR = 1

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class Actor(nn.Module):
    def __init__(self,
    name,           # Name of the network
    state_size,     # Number of input neurons
    action_size,    # Number of output neurons
    hidden_size,    # Number of neurons in hidden layers
    visual = None,  # Visual object
    log_std_min=-20, # Minimum standard deviation of the Gaussian policy
    log_std_max=2   # Maximum standard deviation of the Gaussian policy
    ):
        super(Actor, self).__init__()

        # General information
        self.name = name
        self.visual = visual
        self.iteration = 0

        # log_std_min and log_std_max are used to clamp the standard deviation of the Gaussian policy
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max

        # Define the network
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.mean = nn.Linear(hidden_size, action_size)
        self.log_std = nn.Linear(hidden_size, action_size)

        # Initialize the weights of the network
        self.apply(self.init_weights)

    def init_weights(self, m: torch.nn.Module):
        # Initialize the weights of the network
        if isinstance(m, torch.nn.Linear):
            # Kaiming He initialization
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            m.bias.data.fill_(0.0)
    
    def forward(self, state, visualize=False):
        # Forward pass
        x1 = F.relu(self.fc1(state))
        x2 = F.relu(self.fc2(x1))
        x3 = F.relu(self.fc3(x2))
        # Mean of the Gaussian policy
        mean = self.mean(x3)
        # Clamp the standard deviation of the Gaussian policy
        log_std = self.log_std(x3) 
        log_std = torch.clamp(log_std, min=self.log_std_min, max=self.log_std_max) 
        # Return the mean and the standard deviation of the Gaussian policy
        std = log_std.exp()

        # If visualization is enabled, update the layers
        if visualize and self.visual:
            # Using x as feature visualization

            with torch.no_grad():
                norm = torch.distributions.Normal(mean, std)
                x_t = norm.rsample()
                action = torch.tanh(x_t)

            self.visual.tab_actor_update(actions = action[0],
                                         hidden = [x1, x2, x3],
                                         biases = [self.fc1.bias, self.fc2.bias, self.fc3.bias])

        return mean, std
    
    def sample(self, state, visualize=False):
        # Sample an action from the Gaussian policy
        mean, std = self.forward(state, visualize)
        # Normal distribution
        normal = torch.distributions.Normal(mean, std)
        # Sample an action
        x_t = normal.rsample()
        # Tanh squashing function
        action = torch.tanh(x_t)
        # Log probability of the action
        log_prob = normal.log_prob(x_t).sum(dim=-1)
        log_prob -= (2 * (np.log(2) - x_t - F.softplus(-2 * x_t))).sum(dim=-1)
        # Return the action and the log probability of the action
        return action, log_prob

class Critic(nn.Module):
    def __init__(self,
    name,           # Name of the network 
    state_size,     # Number of input neurons
    action_size,    # Number of output neurons
    hidden_size,    # Number of neurons in hidden layers
    visual = None
    ):
        super(Critic, self).__init__()

        self.name = name
        self.visual = visual
        self.iteration = 0

        self.fc1 = nn.Linear(state_size + action_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.q = nn.Linear(hidden_size, 1)

        self.apply(self.init_weights)
    
    def forward(self, state, action):
        x = torch.cat([state, action], dim=1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        q = self.q(x)
        return q
    
    def visualize_forward(self, sa):

        with torch.no_grad(): # No gradient calculation

            # Forward pass
            x1 = F.relu(self.fc1(sa))
            x2 = F.relu(self.fc2(x1))
            x3 = F.relu(self.fc3(x2))
            # Value of the Q-value network
            value = self.q(x3)
        
        return value, x1, x2, x3, self.fc1.bias, self.fc2.bias, self.fc3.bias
    
    def init_weights(self, m: torch.nn.Module):
        # Initialize the weights of the network
        if isinstance(m, torch.nn.Linear):
            # Kaiming He initialization
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            m.bias.data.fill_(0.0)

class QValue(nn.Module):
    def __init__(self, 
    name,           # Name of the network 
    state_size,     # Number of input neurons
    action_size,    # Number of output neurons
    hidden_size,    # Number of neurons in hidden layers
    visual = None
    ):
        super(QValue, self).__init__()
        
        self.name = name
        self.visual = visual
        self.iteration = 0

        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.val = nn.Linear(hidden_size, 1)

        self.apply(self.init_weights)
    
    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        value = self.val(x)
        return value
    
    def init_weights(self, m: torch.nn.Module):
        # Initialize the weights of the network
        if isinstance(m, torch.nn.Linear):
            # Kaiming He initialization
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            m.bias.data.fill_(0.0)
    

class SAC(BaseAgent):
    def __init__(self, device, algorithm):
        super().__init__(device, algorithm)
        # Set aplha
        self.alpha = 0.2

        # Actor and critic networks
        self.actor = self.create_network(Actor, 'actor')
        self.critic_1 = self.create_network(Critic, 'critic_1')
        self.critic_2 = self.create_network(Critic, 'critic_2')
        # Q-value network
        self.q_value = self.create_network(QValue, 'q_value')
        self.q_value_target = self.create_network(QValue, 'q_value_target')
        self.q_value_target.load_state_dict(self.q_value.state_dict())

        # Optimizers
        self.actor_optimizer = torch.optim.AdamW(self.actor.parameters(), lr=self.learning_rate_actor)
        self.critic_1_optimizer = torch.optim.AdamW(self.critic_1.parameters(), lr=self.learning_rate_critic)
        self.critic_2_optimizer = torch.optim.AdamW(self.critic_2.parameters(), lr=self.learning_rate_critic)
        self.q_value_optimizer = torch.optim.AdamW(self.q_value.parameters(), lr=self.learning_rate_critic)

    def get_action_random(self,state):
        # No get_action_random method in SAC
        return self.get_action(state, is_training=True, step=0, visualize=False)

    def get_action(self, state, is_training, step, visualize=False):
        state = torch.FloatTensor(state).to(self.device).unsqueeze(0)
        action, _ = self.actor.sample(state, visualize)

        # If Visualize is True, update the state of the tab
        if visualize:
            self.visual.tab_state_update(states = state)
            sa = torch.cat([state, action], dim=1)

            # Q1
            value1, x11, x21, x31, b11, b21, b31 = self.critic_1.visualize_forward(sa)

            # Q2
            value2, x12, x22, x32, b12, b22, b32 = self.critic_2.visualize_forward(sa)

            # Update the tab
            self.visual.tab_critic_update(q_values = [value1[0], value2[0]],
                                        hidden = [x11, x21, x31, x12, x22, x32],
                                        biases = [b11, b21, b31, b12, b22, b32])


        return action.detach().cpu().numpy().tolist()[0]
    
    def grad_norm_zero_assert(self, m: torch.nn.Module):
        _p_grad_norm = []
        for p in m.parameters():
            _p_grad_norm.append(p.grad.norm())

        # Assert When p grad norm is all zeros
        if all([x == 0 for x in _p_grad_norm]):
            assert False
    
    def train(self, replaybuffer: ReplayBuffer):

        # Sample a batch of transitions
        batch = replaybuffer.sample(self.batch_size)
        state, action, reward, state_next, done = batch

        # Convert the numpy arrays to torch tensors and move to the device
        state = torch.from_numpy(state).to(self.device)
        action = torch.from_numpy(action).to(self.device)
        reward = torch.from_numpy(reward).to(self.device)
        state_next = torch.from_numpy(state_next).to(self.device)
        done = torch.from_numpy(done).to(self.device)

        # Update Critic networks
        with torch.no_grad():
            # Sample an action from the Gaussian policy
            next_action, next_log_prob = self.actor.sample(state_next)
            next_log_prob = next_log_prob.reshape(-1, 1)
            # Compute the target Q-value
            target_value = self.q_value_target(state_next)
            q_target = reward + (1 - done) * self.discount_factor * (target_value - self.alpha * next_log_prob)

        # Compute the Q-values
        q1 = self.critic_1(state, action)
        q2 = self.critic_2(state, action)

        critic_1_loss = self.loss_function(q1, q_target)
        critic_2_loss = self.loss_function(q2, q_target)

        # Optimize the critic networks
        self.critic_1_optimizer.zero_grad()
        critic_1_loss.backward()
        # Clip the gradients
        nn.utils.clip_grad_norm_(self.critic_1.parameters(), max_norm=2.0, norm_type=2)
        # Assert when the gradients are all zeros
        self.grad_norm_zero_assert(self.critic_1)
        # Update the weights
        self.critic_1_optimizer.step()

        self.critic_2_optimizer.zero_grad()
        critic_2_loss.backward()
        nn.utils.clip_grad_norm_(self.critic_2.parameters(), max_norm=2.0, norm_type=2)
        self.grad_norm_zero_assert(self.critic_2)
        self.critic_2_optimizer.step()

        # Update the Q-value network
        new_action, new_log_prob = self.actor.sample(state)
        new_log_prob = new_log_prob.reshape(-1, 1)
        q1_new = self.critic_1(state, new_action)
        q2_new = self.critic_2(state, new_action)
        q_new = torch.minimum(q1_new, q2_new)
        q_value_target = q_new - self.alpha * new_log_prob

        q_value = self.q_value(state)
        q_value_loss = self.loss_function(q_value, q_value_target.detach())

        # Optimize the Q-value network
        self.q_value_optimizer.zero_grad()
        q_value_loss.backward()
        nn.utils.clip_grad_norm_(self.q_value.parameters(), max_norm=2.0, norm_type=2)
        self.grad_norm_zero_assert(self.q_value)
        self.q_value_optimizer.step()

        # Update the Actor network
        actor_loss = (self.alpha * new_log_prob - q_new).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=2.0, norm_type=2)
        self.grad_norm_zero_assert(self.actor)
        self.actor_optimizer.step()

        # Soft update of the target Q-value network
        self.soft_update(target=self.q_value_target, source=self.q_value, tau=self.tau)

        # Return the critic loss and the actor loss
        avg_critic_loss = (critic_1_loss + critic_2_loss) / 2
        return [avg_critic_loss.detach().cpu(), actor_loss.detach().cpu()]
        