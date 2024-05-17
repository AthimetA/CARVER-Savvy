#!/usr/bin/env python

from settings.constparams import ENABLE_VISUAL, NUM_SCAN_SAMPLES

if ENABLE_VISUAL:
    from PyQt5 import QtWidgets, QtCore
    import pyqtgraph as pg
    import numpy as np
    import time

    import torch

    pg.setConfigOptions(antialias=False)

    class DrlVisual(pg.GraphicsLayoutWidget):
        def __init__(self, state_size, hidden_size):
            super().__init__(None)
            self.show()
            self.resize(1980, 1200)

            self.state_size = state_size
            self.hidden_sizes = [hidden_size, hidden_size]

            self.mainLayout = QtWidgets.QVBoxLayout()
            self.setLayout(self.mainLayout)

            # -------------- States Plot -------------- #
            # Add plot to the layout
            self.plot_item_states = self.addPlot(title="States", row = 0, col = 0, colspan=3)
            # Set the range of the plot
            self.plot_item_states.setXRange(-1, self.state_size, padding=0)
            self.plot_item_states.setYRange(-1, 1, padding=0)
            # Add the bar graph to the plot
            self.bar_graph_states = pg.BarGraphItem(x=range(self.state_size), height = np.zeros(self.state_size), width=0.8)
            self.plot_item_states.addItem(self.bar_graph_states)

            # ----------- Details states ------------ #
            # Distance to Goal
            self.dtg_item = self.addPlot(title="Distance to Goal", row = 1, col = 0, colspan=1)
            self.dtg_item.setXRange(-1, 1, padding=0)
            self.dtg_item.setYRange(-1, 1, padding=0)
            self.bar_graph_dtg = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.dtg_item.addItem(self.bar_graph_dtg)
            # Angle to Goal
            self.atg_item = self.addPlot(title="Angle to Goal", row = 1, col = 1, colspan=1)
            self.atg_item.setXRange(-1, 1, padding=0)
            self.atg_item.setYRange(-1, 1, padding=0)
            self.bar_graph_atg = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.bar_graph_atg.setRotation(-90)
            self.atg_item.addItem(self.bar_graph_atg)
            # Theta
            self.theta_item = self.addPlot(title="Theta", row = 1, col = 2, colspan=1)
            self.theta_item.setXRange(-1, 1, padding=0)
            self.theta_item.setYRange(-1, 1, padding=0)
            self.bar_graph_theta = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.theta_item.addItem(self.bar_graph_theta)
            # Robot linear velocity
            self.linear_item = self.addPlot(title="Linear Velocity", row = 2, col = 0, colspan=1)
            self.linear_item.setXRange(-1, 1, padding=0)
            self.linear_item.setYRange(-1, 1, padding=0)
            self.bar_graph_linear = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.linear_item.addItem(self.bar_graph_linear)
            # Robot angular velocity
            self.angular_item = self.addPlot(title="Angular Velocity", row = 2, col = 1, colspan=1)
            self.angular_item.setXRange(-1, 1, padding=0)
            self.angular_item.setYRange(-1, 1, padding=0)
            self.bar_graph_angular = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.bar_graph_angular.setRotation(-90)
            self.angular_item.addItem(self.bar_graph_angular)
            # Obstacle distance
            self.obstacle_item = self.addPlot(title="Obstacle Distance", row = 2, col = 2, colspan=1)
            self.obstacle_item.setXRange(-1, 1, padding=0)
            self.obstacle_item.setYRange(-1, 1, padding=0)
            self.bar_graph_obstacle = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.obstacle_item.addItem(self.bar_graph_obstacle)
            # Obstacle angle
            self.obstacle_angle_item = self.addPlot(title="Obstacle Angle", row = 3, col = 2, colspan=1)
            self.obstacle_angle_item.setXRange(-1, 1, padding=0)
            self.obstacle_angle_item.setYRange(-1, 1, padding=0)
            self.bar_graph_obstacle_angle = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.bar_graph_obstacle_angle.setRotation(-90)
            self.obstacle_angle_item.addItem(self.bar_graph_obstacle_angle)
            # Obstacle linear velocity
            self.obstacle_linear_item = self.addPlot(title="Obstacle Linear Velocity", row = 3, col = 0, colspan=1)
            self.obstacle_linear_item.setXRange(-1, 1, padding=0)
            self.obstacle_linear_item.setYRange(-1, 1, padding=0)
            self.bar_graph_obstacle_linear = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.obstacle_linear_item.addItem(self.bar_graph_obstacle_linear)
            # Obstacle angular velocity
            self.obstacle_angular_item = self.addPlot(title="Obstacle Angular Velocity", row = 3, col = 1, colspan=1)
            self.obstacle_angular_item.setXRange(-1, 1, padding=0)
            self.obstacle_angular_item.setYRange(-1, 1, padding=0)
            self.bar_graph_obstacle_angular = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.bar_graph_obstacle_angular.setRotation(-90)
            self.obstacle_angular_item.addItem(self.bar_graph_obstacle_angular)
            # Last action linear velocity
            self.last_action_linear_item = self.addPlot(title="Last Action Linear", row = 0, col = 3, colspan=1)
            self.last_action_linear_item.setXRange(-1, 1, padding=0)
            self.last_action_linear_item.setYRange(-1, 1, padding=0)
            self.bar_graph_last_action_linear = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.last_action_linear_item.addItem(self.bar_graph_last_action_linear)
            # Last action angular velocity
            self.last_action_angular_item = self.addPlot(title="Last Action Angular", row = 0, col = 4, colspan=1)
            self.last_action_angular_item.setXRange(-1, 1, padding=0)
            self.last_action_angular_item.setYRange(-1, 1, padding=0)
            self.bar_graph_last_action_angular = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.bar_graph_last_action_angular.setRotation(-90)
            self.last_action_angular_item.addItem(self.bar_graph_last_action_angular)

            # -------------- Hidden Layers Output and Bias -------------- #
            self.hidden_plot_items = []
            self.hidden_bar_graphs = []
            self.hidden_line_plots = []
            i = 0
            for hidden_size in self.hidden_sizes:
                plot_item = self.addPlot(title=f"Hidden layer {i}", row = i+1, col = 3, colspan=3)
                plot_item.setXRange(-1, hidden_size, padding=0)
                plot_item.setYRange(-0.2, 1.3, padding=0)

                bar_graph = pg.BarGraphItem(x=range(hidden_size), height=np.zeros(hidden_size), width=0.8)
                plot_item.addItem(bar_graph)

                line_plot = plot_item.plot(x=range(hidden_size), brush='r', symbol='x', symbolPen='r')
                line_plot.setPen(style=QtCore.Qt.NoPen)
                line_plot.setSymbolSize(5)

                self.hidden_bar_graphs.append(bar_graph)
                self.hidden_plot_items.append(plot_item)
                self.hidden_line_plots.append(line_plot)
                i += 1

            # -------------- Output layers -------------- #
            # Action Linear
            self.plot_item_action_linear = self.addPlot(title="Action Linear", row = 3, col = 3, colspan=1)
            self.plot_item_action_linear.setXRange(-20, 20, padding=0)
            self.plot_item_action_linear.setYRange(-1, 1, padding=0)
            self.bar_graph_action_linear = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.plot_item_action_linear.addItem(self.bar_graph_action_linear)

            # Action Angular
            self.plot_item_action_angular = self.addPlot(title="Action Angular", row = 3, col = 4, colspan=1)
            self.plot_item_action_angular.setXRange(-1, 1, padding=0)
            self.plot_item_action_angular.setYRange(-1.5, 1.5, padding=0)
            self.bar_graph_action_angular = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.bar_graph_action_angular.setRotation(-90)
            self.plot_item_action_angular.addItem(self.bar_graph_action_angular)

            # Accumulated Reward
            self.plot_item_reward = self.addPlot(title="Accumlated Reward", row = 3, col = 5, colspan=1)
            self.plot_item_reward.setXRange(-1, 1, padding=0)
            self.plot_item_reward.setYRange(-5000, 1000, padding=0)
            self.bar_graph_reward = pg.BarGraphItem(x=range(1), height=[0], width=0.5)
            self.plot_item_reward.addItem(self.bar_graph_reward)

            self.iteration = 0

        def prepare_data(self, tensor : torch.Tensor):
            return tensor.squeeze().flip(0).detach().cpu()

        def update_layers(self, states, actions, hidden, biases):
            # States data
            states_data = self.prepare_data(states)

            # Update the states
            self.bar_graph_states.setOpts(height=states_data)

            # Update the details states start from the end of the states_data
            self.bar_graph_dtg.setOpts(height=[states_data[10]])
            self.bar_graph_atg.setOpts(height=[states_data[9]])
            self.bar_graph_theta.setOpts(height=[states_data[8]])
            self.bar_graph_linear.setOpts(height=[states_data[7]])
            self.bar_graph_angular.setOpts(height=[states_data[6]])
            self.bar_graph_obstacle.setOpts(height=[states_data[5]])
            self.bar_graph_obstacle_angle.setOpts(height=[states_data[4]])
            self.bar_graph_obstacle_linear.setOpts(height=[states_data[3]])
            self.bar_graph_obstacle_angular.setOpts(height=[states_data[2]])
            self.bar_graph_last_action_linear.setOpts(height=[states_data[1]])
            self.bar_graph_last_action_angular.setOpts(height=[states_data[0]])

            # Update the Actions
            actions = actions.detach().cpu().numpy().tolist()
            self.bar_graph_action_linear.setOpts(height=[actions[0]])
            self.bar_graph_action_angular.setOpts(height=[actions[1]])
            for i in range(len(hidden)):
                self.hidden_bar_graphs[i].setOpts(height=self.prepare_data(hidden[i]))
            pg.QtGui.QGuiApplication.processEvents()
            if self.iteration % 100 == 0:
                self.update_bias(biases)
            self.iteration += 1

        def update_bias(self, biases):
            for i in range(len(biases)):
                self.hidden_line_plots[i].setData(y=self.prepare_data(biases[i]))

        def update_reward(self, acc_reward):
            self.bar_graph_reward.setOpts(height=[acc_reward])
            if acc_reward > 0:
                self.bar_graph_reward.setOpts(brush='g')
            else:
                self.bar_graph_reward.setOpts(brush='r')


    def test():
        win = DrlVisual(30, 512)
        i = 200
        while (i):
            starttime = time.perf_counter()
            vals1 = np.random.rand(30)
            vals2 = np.random.rand(2)
            vals3 = [np.random.rand(512)] * 2
            vals4 = [np.random.rand(512)] * 2
            win.update_layers(vals1, vals2, vals3, vals4)
            win.update_bias(vals4)
            win.update_reward(i*10*vals1[0])
            print(f"time: {time.perf_counter() - starttime}")
            i -= 1
    if __name__ == "__main__":
        test()