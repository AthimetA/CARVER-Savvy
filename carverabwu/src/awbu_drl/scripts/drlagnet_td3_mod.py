import numpy as np
import copy

import torch
import torch.nn.functional as F
import torch.nn as nn

from settings.constparams import POLICY_NOISE, POLICY_NOISE_CLIP, POLICY_UPDATE_FREQUENCY

from drlagent_off_policy_agent import OffPolicyAgent, Network, OUNoise

LINEAR = 0
ANGULAR = 1

# Implementation of Twin Delayed Deep Deterministic Policy Gradients (TD3)
# Paper: https://arxiv.org/abs/1802.09477
# Original implementation: https://github.com/sfujim/TD3/blob/master/TD3.py [Stable Baselines original implementation]

class Actor(Network):
    def __init__(self,
    name,           # Name of the network
    state_size,     # Number of input neurons
    action_size,    # Number of output neurons
    hidden_size     # Number of neurons in hidden layers
    ):
        super(Actor, self).__init__(name)
        # Layer Definition
        self.fa1 = nn.Linear(state_size, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)

        # Initialize weights
        # Using Kaiming initialization
        self.apply(super().init_weights)

    def forward(self, states, visualize=False):
        # Forward pass
        x1 = torch.relu(self.fa1(states))
        x2 = torch.relu(self.fa2(x1))
        action = torch.tanh(self.fa3(x2))

        # If visualization is enabled, update the layers
        if visualize and self.visual:
            # Using x1 and x2 as features for visualization
            # self.visual.update_layers(states, action, [x1, x2], [self.fa1.bias, self.fa2.bias])
            self.visual.tab_actor_update(actions = action,
                                         hidden = [x1, x2],
                                         biases = [self.fa1.bias, self.fa2.bias])

        return action

class Critic(Network):
    def __init__(self,
    name,           # Name of the network 
    state_size,     # Number of input neurons
    action_size,    # Number of output neurons
    hidden_size     # Number of neurons in hidden layers
    ):
        super(Critic, self).__init__(name)

        # Q1 Architecture
        self.l1 = nn.Linear(state_size + action_size, hidden_size)
        self.l2 = nn.Linear(hidden_size, hidden_size)
        self.l3 = nn.Linear(hidden_size, 1)

        # Q2 Architecture
        self.l5 = nn.Linear(state_size + action_size, hidden_size)
        self.l6 = nn.Linear(hidden_size, hidden_size)
        self.l7 = nn.Linear(hidden_size, 1)

        self.apply(super().init_weights)

    def forward(self, states, actions):
        
        # Concatenate the states and actions
        sa = torch.cat((states, actions), dim=1)

        # Q1 forward pass
        x1 = torch.relu(self.l1(sa))
        x2 = torch.relu(self.l2(x1))
        q1 = self.l3(x2)

        # Q2 forward pass
        x5 = torch.relu(self.l5(sa))
        x6 = torch.relu(self.l6(x5))
        q2 = self.l7(x6)

        return q1, q2


    def Q1_forward(self, states, actions):
        
        # Concatenate the states and actions
        sa = torch.cat((states, actions), dim=1)

        # Q1 forward pass
        x1 = torch.relu(self.l1(sa))
        x2 = torch.relu(self.l2(x1))
        q1 = self.l3(x2)

        return q1

class TD3(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)

        # DRL parameters
        self.noise = OUNoise(action_space=self.action_size, max_sigma=0.9, min_sigma=0.1, decay_period=700_000)

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
        self.critic_optimizer.step()

        # Delayed policy updates
        if self.iteration % self.policy_freq == 0:
            # optimize actor
            loss_actor = -1 * self.critic.Q1_forward(state, self.actor(state)).mean()

            # Optimize the actor
            self.actor_optimizer.zero_grad()
            loss_actor.backward()
            self.actor_optimizer.step()

            # Update the frozen target models
            self.soft_update(self.actor_target, self.actor, self.tau)
            self.soft_update(self.critic_target, self.critic, self.tau)

            # Visualize the network
            self.last_actor_loss = loss_actor.mean().detach().cpu()
        return [loss_critic.mean().detach().cpu(), self.last_actor_loss]
    