import numpy as np
import copy

import torch
import torch.nn.functional as F
import torch.nn as nn

from settings.constparams import POLICY_NOISE, POLICY_NOISE_CLIP, POLICY_UPDATE_FREQUENCY

from drlagent_off_policy_agent import OffPolicyAgent, OUNoise

from abc import ABC, abstractmethod

LINEAR = 0
ANGULAR = 1

# Implementation of Twin Delayed Deep Deterministic Policy Gradients (TD3)
# Paper: https://arxiv.org/abs/1802.09477
# Original implementation: https://github.com/sfujim/TD3/blob/master/TD3.py [Stable Baselines original implementation]

class Actor(torch.nn.Module, ABC):
    def __init__(self,
    name,           # Name of the network
    state_size,     # Number of input neurons
    action_size,    # Number of output neurons
    hidden_size,     # Number of neurons in hidden layers
    visual= None
    ):
        super(Actor, self).__init__()
        self.name = name
        self.visual = visual
        self.iteration = 0

        # Layer Definition
        self.fa1 = nn.Linear(state_size, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, hidden_size)
        self.fa4 = nn.Linear(hidden_size, action_size)

        # Initialize weights
        # Using Kaiming initialization
        self.apply(self.init_weights)

    def init_weights(self, m: torch.nn.Module):
        # Initialize the weights of the network
        if isinstance(m, torch.nn.Linear):
            # Kaiming He initialization
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            m.bias.data.fill_(0.0)


    def forward(self, states, visualize=False):
        # Forward pass
        x1 = torch.relu(self.fa1(states))
        x2 = torch.relu(self.fa2(x1))
        x3 = torch.relu(self.fa3(x2))
        action = torch.tanh(self.fa4(x3))

        # If visualization is enabled, update the layers
        if visualize and self.visual:
            # Using x as feature visualization
            self.visual.tab_actor_update(actions = action,
                                         hidden = [x1, x2, x3],
                                         biases = [self.fa1.bias, self.fa2.bias, self.fa3.bias])

        return action

class Critic(torch.nn.Module, ABC):
    def __init__(self,
    name,           # Name of the network 
    state_size,     # Number of input neurons
    action_size,    # Number of output neurons
    hidden_size,     # Number of neurons in hidden layers
    visual = None
    ):
        super(Critic, self).__init__()
        self.name = name
        self.visual = visual
        self.iteration = 0

        # Q1 Architecture
        self.l01 = nn.Linear(state_size + action_size, hidden_size)
        self.l02 = nn.Linear(hidden_size, hidden_size)
        self.l03 = nn.Linear(hidden_size, hidden_size)
        self.l04 = nn.Linear(hidden_size, 1)

        # Q2 Architecture
        self.l11 = nn.Linear(state_size + action_size, hidden_size)
        self.l12 = nn.Linear(hidden_size, hidden_size)
        self.l13 = nn.Linear(hidden_size, hidden_size)
        self.l14 = nn.Linear(hidden_size, 1)

        # Initialize weights
        # Using Kaiming initialization
        self.apply(self.init_weights)

    def init_weights(self, m: torch.nn.Module):
        # Initialize the weights of the network
        if isinstance(m, torch.nn.Linear):
            # Kaiming He initialization
            torch.nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            m.bias.data.fill_(0.0)

    def forward(self, states, actions, visualize=False) -> torch.Tensor:
        
        # Concatenate the states and actions
        sa = torch.cat((states, actions), dim=1)

        # Q1 forward pass
        x01 = torch.relu(self.l01(sa))
        x02 = torch.relu(self.l02(x01))
        x03 = torch.relu(self.l03(x02))
        q1 = self.l04(x03)

        # Q2 forward pass
        x11 = torch.relu(self.l11(sa))
        x12 = torch.relu(self.l12(x11))
        x13 = torch.relu(self.l13(x12))
        q2 = self.l14(x13)

        return q1, q2


    def Q1_forward(self, states, actions) -> torch.Tensor:
        
        # Concatenate the states and actions
        sa = torch.cat((states, actions), dim=1)

        # Q1 forward pass
        x01 = torch.relu(self.l01(sa))
        x02 = torch.relu(self.l02(x01))
        x03 = torch.relu(self.l03(x02))
        q1 = self.l04(x03)

        return q1
    
    def visualize_forward(self, sa):

        with torch.no_grad(): # No gradient calculation

            # Q1 forward pass
            x01 = torch.relu(self.l01(sa))
            x02 = torch.relu(self.l02(x01))
            x03 = torch.relu(self.l03(x02))
            q1 = self.l04(x03)
            
            # Q2 forward pass
            x11 = torch.relu(self.l11(sa))
            x12 = torch.relu(self.l12(x11))
            x13 = torch.relu(self.l13(x12))
            q2 = self.l14(x13)

        self.visual.tab_critic_update(q_values = [q1, q2],
                                    hidden = [x01, x02, x03, x11, x12, x13],
                                    biases = [self.l01.bias, self.l02.bias, self.l03.bias, self.l11.bias, self.l12.bias, self.l13.bias])


class TD3(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)

        # DRL parameters
        self.noise = OUNoise(action_space=self.action_size, max_sigma=0.3, min_sigma=0.1, decay_period=1_000_000)

        # TD3 parameters
        self.policy_noise   = POLICY_NOISE
        self.noise_clip     = POLICY_NOISE_CLIP
        self.policy_freq    = POLICY_UPDATE_FREQUENCY

        self.last_actor_loss = 0

        # Actor and Target Actor
        self.actor = self.create_network(Actor, 'actor')
        self.actor_target = self.create_network(Actor, 'target_actor')
        # Actor optimizer
        self.actor_optimizer = self.create_optimizer(self.actor)

        # Critic and Target Critic
        self.critic = self.create_network(Critic, 'critic')
        self.critic_target = self.create_network(Critic, 'target_critic')
        # Critic optimizer
        self.critic_optimizer = self.create_optimizer(self.critic)

        self.hard_update(self.actor_target, self.actor)
        self.hard_update(self.critic_target, self.critic)

    def get_action_with_epsilon_greedy(self, state, is_training, step, visualize=False):
        if is_training and np.random.rand() <= self.epsilon:
            return self.get_action_random()
        else:
            return self.get_action(state, is_training, step, visualize)

    def get_action(self, state, is_training, step, visualize=False):
        state = torch.from_numpy(np.asarray(state, np.float32)).to(self.device)
        action = self.actor(state, visualize)

        if visualize:
            self.visual.tab_state_update(states = state)
            sa = torch.cat((state, action), dim=0)
            self.critic.visualize_forward(sa)

        if is_training:
            noise = torch.from_numpy(copy.deepcopy(self.noise.get_noise(step))).to(self.device)
            # print(f'Action: {action.cpu().data.numpy()}, Noise: {noise.cpu().data.numpy()}')
            action = torch.clamp(torch.add(action, noise), -1.0, 1.0)
            # print(f'Action after noise: {action.cpu().data.numpy()}')
            # print('-' * 50)
        return action.detach().cpu().data.numpy().tolist()

    def get_action_random(self):
        return [np.clip(np.random.uniform(-1.0, 1.0), -1.0, 1.0)] * self.action_size

    def train(self, state, action, reward, state_next, done):

        with torch.no_grad():
            # Select action according to policy and add clipped noise
            noise = (
                 torch.randn_like(action) * self.policy_noise
            ).clamp(-self.noise_clip, self.noise_clip)

            action_next = (
                 self.actor_target(state_next) + noise
            ).clamp(-1.0, 1.0)

            # Compute the target Q value
            Q1_next, Q2_next = self.critic_target(state_next, action_next)
            # Take the minimum of the two Q values
            Q_next = torch.min(Q1_next, Q2_next)
            # Compute the target Q value
            Q_target = reward + (1 - done) * self.discount_factor * Q_next

        # Get current Q estimates
        Q1, Q2 = self.critic(state, action)

        # Compute critic loss
        loss_critic = self.loss_function(Q1, Q_target) + self.loss_function(Q2, Q_target)

        # Optimize the critic
        self.critic_optimizer.zero_grad()
        loss_critic.backward()
        # Clip the gradients
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=1.0, norm_type=2)
        self.critic_optimizer.step()

        # Delayed policy updates
        if self.iteration % self.policy_freq == 0:
            
            # optimize actor
            loss_actor = -1 * self.critic.Q1_forward(state, self.actor(state)).mean()

            # Optimize the actor
            self.actor_optimizer.zero_grad()
            loss_actor.backward()
            # Clip the gradients
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=1.0, norm_type=2)
            self.actor_optimizer.step()

            # Update the frozen target models
            self.soft_update(self.actor_target, self.actor, self.tau)
            self.soft_update(self.critic_target, self.critic, self.tau)

            # Visualize the network
            self.last_actor_loss = loss_actor.mean().detach().cpu()
        return [loss_critic.mean().detach().cpu(), self.last_actor_loss]
    