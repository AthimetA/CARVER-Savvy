import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

matplotlib.use('TkAgg')
UNKNOWN = 0
SUCCESS = 1
COLLISION = 2
TIMEOUT = 3
TUMBLE = 4
RESULTS_NUM = 5

class Test_Graph():
    def __init__(self, session_dir, first_episode=0, continue_graph=True):
        plt.ion()  # Enable interactive mode
        self.first_episode = first_episode
        self.continue_graph = continue_graph
        self.session_dir = session_dir
        self.legend_labels = ['Unknown', 'Success', 'Collision', 'Timeout', 'Tumble']
        self.legend_colors = ['b', 'g', 'r', 'c', 'm']
        self.outcome_histories = []
        self.outcome_list = []
        self.global_steps = 0
        self.data_outcome_history = []
        self.data_rewards = []
        self.data_loss_critic = []
        self.data_loss_actor = []
        self.SOCIAL_SCORE_LIST = []
        self.EGO_SCORE_LIST = []
        self.ARIVAL_TIME = []
        self.graphdata = [self.global_steps, self.data_outcome_history,
                          self.EGO_SCORE_LIST, self.SOCIAL_SCORE_LIST, self.ARIVAL_TIME]
        self.fig, self.ax = plt.subplots(2, 2)
        self.fig.set_size_inches(18.5, 10.5)
        titles = ['outcomes', 'EGO score (Success only)', 'Arrival time', 'Social score (Success only)']
        for i in range(4):
            ax = self.ax[int(i / 2)][int(i % 2 != 0)]
            ax.set_title(titles[i])
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        self.legend_set = False
        self.test_outcome = [0] * RESULTS_NUM
        

    def clear_graph(self):
        self.outcome_list = []
        self.data_outcome_history = []
        self.data_rewards = []
        self.data_loss_critic = []
        self.data_loss_actor = []
        self.graphdata = [self.global_steps, self.data_outcome_history,
                          self.EGO_SCORE_LIST, self.SOCIAL_SCORE_LIST, self.ARIVAL_TIME]
        self.outcome_histories = []
        self.SOCIAL_SCORE_LIST = []
        self.EGO_SCORE_LIST = []
        self.ARIVAL_TIME = []
        for ax_row in self.ax:
            for ax in ax_row:
                ax.clear()
        self.legend_set = False

    def set_graphdata(self, graphdata, episode):
        [self.global_steps, self.data_outcome_history,
         self.EGO_SCORE_LIST, self.SOCIAL_SCORE_LIST, self.ARIVAL_TIME] = [
            graphdata[i] for i in range(len(self.graphdata))]
        self.graphdata = [self.global_steps, self.data_outcome_history,
                          self.EGO_SCORE_LIST, self.SOCIAL_SCORE_LIST, self.ARIVAL_TIME]
        if not self.continue_graph:
            self.clear_graph()
        self.draw_plots(episode)
        return self.global_steps

    def update_data(self, step, global_steps, outcome, k_time, m_time, total_time):
        self.global_steps = global_steps
        self.outcome_list.append(outcome)
        EGO_SCORE = (1 - (k_time / total_time)) * 100
        SOCIAL_SCORE = (1 - (m_time / total_time)) * 100
        if outcome == SUCCESS:
            self.EGO_SCORE_LIST.append(EGO_SCORE)
            self.SOCIAL_SCORE_LIST.append(SOCIAL_SCORE)
        self.ARIVAL_TIME.append(total_time)
        self.graphdata = [self.global_steps, self.data_outcome_history,
                          self.EGO_SCORE_LIST, self.SOCIAL_SCORE_LIST, self.ARIVAL_TIME]
        
        self.test_outcome[outcome] += 1
        success_count = self.test_outcome[SUCCESS]
        
        print(f"Successes: {self.test_outcome[SUCCESS]} ({self.test_outcome[SUCCESS]/global_steps:.2%}), "
            f"collision: {self.test_outcome[COLLISION]} ({self.test_outcome[COLLISION]/global_steps:.2%}), "
            f"timeouts: {self.test_outcome[TIMEOUT]}, ({self.test_outcome[TIMEOUT]/global_steps:.2%}), ")
            # f"tumbles: {self.test_outcome[TUMBLE]}, ({self.test_outcome[TUMBLE]/self.test_entry:.2%}), ")

        if success_count > 0:
            print(
                    f"AVG EGO_SCORE: {sum(self.EGO_SCORE_LIST)/success_count:.2%} "
                    f"AVG SOCIAL_SCORE: {sum(self.SOCIAL_SCORE_LIST)/success_count:.2%} "
                    f"AVG ARIVAL TIME: {sum(self.ARIVAL_TIME)/len(self.ARIVAL_TIME):.2f} "
                    )


    def draw_plots(self, ep, save=False):
        if not self.continue_graph:
            episode = ep - self.first_episode
        else:
            episode = ep

        xaxis = np.array(range(1, episode + 1))

        # Plot outcome history
        for idx in range(len(self.outcome_list)):
            if idx == 0:
                self.outcome_histories = [[0], [0], [0], [0], [0]]
                self.outcome_histories[self.outcome_list[0]][0] += 1
            else:
                for outcome_history in self.outcome_histories:
                    outcome_history.append(outcome_history[-1])
                self.outcome_histories[self.outcome_list[idx]][-1] += 1

        if len(self.outcome_list) > 0:
            i = 0
            for outcome_history in self.outcome_histories:
                self.ax[0][0].plot(xaxis, outcome_history, color=self.legend_colors[i], label=self.legend_labels[i])
                i += 1
            if not self.legend_set:
                self.ax[0][0].legend()
                self.legend_set = True

        # Plot SUCCESS % history
        if len(self.outcome_list) > 0:
            success_rate = np.sum(np.array(self.outcome_list) == SUCCESS) / len(self.outcome_list) * 100
            self.ax[0][0].text(0.0, 1.0, f'SUCCESS RATE: {success_rate:.2f}%', transform=self.ax[0][0].transAxes, fontsize=9, verticalalignment='top', bbox=dict(facecolor='white', alpha=1))

        S_axis = np.array(range(1, len(self.EGO_SCORE_LIST) + 1))
        # Plot EGO SCORE
        y = np.array(self.EGO_SCORE_LIST)
        self.ax[0][1].plot(S_axis, y)

        if len(self.EGO_SCORE_LIST) > 0:
                avg_ego_score = np.mean(self.EGO_SCORE_LIST)
                self.ax[0][1].text(0.0, 1.0, f'AVG EGO SCORE: {avg_ego_score:.2f}', transform=self.ax[0][1].transAxes,
                                fontsize=9, verticalalignment='top', bbox=dict(facecolor='white', alpha=1))

        # Plot SOCIAL_SCORE
        y = np.array(self.SOCIAL_SCORE_LIST)
        self.ax[1][1].plot(S_axis, y)

        if len(self.SOCIAL_SCORE_LIST) > 0:
            avg_social_score = np.mean(self.SOCIAL_SCORE_LIST)
            self.ax[1][1].text(0.0, 1.0, f'AVG SOCIAL SCORE: {avg_social_score:.2f}', transform=self.ax[1][1].transAxes,
                            fontsize=9, verticalalignment='top', bbox=dict(facecolor='white', alpha=1))

        

        # Plot Arrival time
        y = np.array(self.ARIVAL_TIME)
        self.ax[1][0].plot(xaxis, y)

        if len(self.ARIVAL_TIME) > 0:
            avg_arrival_time = np.mean(self.ARIVAL_TIME)
            self.ax[1][0].text(0.0, 1.0, f'AVG ARRIVAL TIME: {avg_arrival_time:.2f}', transform=self.ax[1][0].transAxes,
                            fontsize=9, verticalalignment='top', bbox=dict(facecolor='white', alpha=1))


        plt.draw()
        plt.pause(0.2)

        if save:
            plt.savefig(os.path.join(self.session_dir, "_figure_test.png"))

# Ensure the instance is created and the methods are called appropriately
# Example usage:
# graph = Test_Graph(session_dir="path/to/session_dir")
# graph.update_data(step, global_steps, outcome, k_time, m_time, total_time)
# graph.set_graphdata(graphdata, episode)
# graph.draw_plots(ep)
