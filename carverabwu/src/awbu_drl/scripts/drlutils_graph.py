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

import os
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
from settings.constparams import GRAPH_DRAW_INTERVAL, GRAPH_AVERAGE_REWARD, SUCCESS
from matplotlib.ticker import MaxNLocator

matplotlib.use('TkAgg')
class Graph():
    def __init__(self, session_dir, first_episode = 0, continue_graph = True):
        plt.show()

        self.first_episode = first_episode
        self.continue_graph = continue_graph

        self.session_dir = session_dir
        self.legend_labels = ['Unknown', 'Success', 'Collision', 'Timeout', 'Tumble']
        self.legend_colors = ['b', 'g', 'r', 'c', 'm']

        self.outcome_histories = []

        self.global_steps = 0
        self.data_outcome_history = []
        self.data_rewards = []
        self.data_loss_critic = []
        self.data_loss_actor = []
        self.graphdata = [self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor]

        self.fig, self.ax = plt.subplots(2, 2)
        self.fig.set_size_inches(18.5, 10.5)

        titles = ['outcomes', 'avg critic loss over episode', 'avg actor loss over episode', 'avg reward over 10 episodes']
        for i in range(4):
            ax = self.ax[int(i/2)][int(i%2!=0)]
            ax.set_title(titles[i])
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        self.legend_set = False

    def clear_graph(self):
        self.data_outcome_history = []
        self.data_rewards = []
        self.data_loss_critic = []
        self.data_loss_actor = []
        self.graphdata = [self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor]

        self.outcome_histories = []

        self.ax[0][0].clear()
        self.ax[0][1].clear()
        self.ax[1][0].clear()
        self.ax[1][1].clear()

        self.legend_set = False

    def set_graphdata(self, graphdata, episode):
        self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor = [graphdata[i] for i in range(len(self.graphdata))]
        self.graphdata = [self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor]
        if not self.continue_graph:
            self.clear_graph()
        self.draw_plots(episode)
        return self.global_steps

    def update_data(self, step, global_steps, outcome, reward_sum, loss_critic_sum, loss_actor_sum):
        self.global_steps = global_steps
        self.data_outcome_history.append(outcome)
        self.data_rewards.append(reward_sum)
        self.data_loss_critic.append(loss_critic_sum / step)
        self.data_loss_actor.append(loss_actor_sum / step)
        self.graphdata = [self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor]

    def draw_plots(self, ep):
        
        if not self.continue_graph:
            episode = ep - self.first_episode
        else:
            episode = ep

        xaxis = np.array(range(1, episode + 1))

        # Plot outcome history
        for idx in range(len(self.data_outcome_history)):
            if idx == 0:
                self.outcome_histories = [[0],[0],[0],[0],[0]]
                self.outcome_histories[self.data_outcome_history[0]][0] += 1
            else:
                for outcome_history in self.outcome_histories:
                    outcome_history.append(outcome_history[-1])
                self.outcome_histories[self.data_outcome_history[idx]][-1] += 1

        if len(self.data_outcome_history) > 0:
            i = 0
            for outcome_history in self.outcome_histories:
                self.ax[0][0].plot(xaxis, outcome_history, color=self.legend_colors[i], label=self.legend_labels[i])
                i += 1
            if not self.legend_set:
                # self.ax[0][0].legend()
                self.legend_set = True

        # Plot critic loss
        y = np.array(self.data_loss_critic)
        self.ax[0][1].plot(xaxis, y)

        # Plot actor loss
        y = np.array(self.data_loss_actor)
        self.ax[1][0].plot(xaxis, y)

        # Plot average reward
        count = int(episode / GRAPH_AVERAGE_REWARD)
        if count > 0:
            xaxis = np.array(range(GRAPH_AVERAGE_REWARD, episode+1, GRAPH_AVERAGE_REWARD))
            averages = list()
            for i in range(count):
                avg_sum = 0
                for j in range(GRAPH_AVERAGE_REWARD):
                    avg_sum += self.data_rewards[i * GRAPH_AVERAGE_REWARD + j]
                averages.append(avg_sum / GRAPH_AVERAGE_REWARD)
            y = np.array(averages)
            self.ax[1][1].plot(xaxis, y)

        plt.draw()
        plt.pause(0.2)
        plt.savefig(os.path.join(self.session_dir, "_figure.png"))

    def get_success_count(self):
        suc = self.data_outcome_history[-GRAPH_DRAW_INTERVAL:]
        return suc.count(SUCCESS)

    def get_reward_average(self):
        rew = self.data_rewards[-GRAPH_DRAW_INTERVAL:]
        return sum(rew) / len(rew)