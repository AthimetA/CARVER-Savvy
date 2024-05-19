#!/usr/bin/env python

from settings.constparams import ENABLE_VISUAL, NUM_SCAN_SAMPLES

if ENABLE_VISUAL:
    from PyQt5 import QtWidgets, QtCore
    import pyqtgraph as pg
    import numpy as np
    import time

    import torch

    pg.setConfigOptions(antialias=False)

    class DrlVisual(QtWidgets.QMainWindow):
        def __init__(self, state_size, hidden_size):
            super().__init__(None)
            # Set the window 
            self.show()
            self.resize(1980, 1200)

            self.state_size = state_size
            self.hidden_sizes = [hidden_size, hidden_size, hidden_size]

            # Create the main layout
            self.central_widget = QtWidgets.QWidget()
            self.setCentralWidget(self.central_widget)
            self.mainLayout = QtWidgets.QVBoxLayout(self.central_widget)

            # Add tab widget
            self.tab_widget = QtWidgets.QTabWidget()
            self.mainLayout.addWidget(self.tab_widget)

            # Create the tab for the visualizer
            # DRL State tab
            self.tab_state = QtWidgets.QWidget()
            self.tab_widget.addTab(self.tab_state, "DRL State Visualizer")
            # Actor tab
            self.tab_actor = QtWidgets.QWidget()
            self.tab_widget.addTab(self.tab_actor, "Actor Visualizer")
            # Critic tab
            self.tab_critic = QtWidgets.QWidget()
            self.tab_widget.addTab(self.tab_critic, "Critic Visualizer")

            # Tab Initialization
            self.init_tab_state()
            self.init_tab_actor()
            self.init_tab_critic()
            
            self.actor_iteration = 0
            self.critic_iteration = 0

        def init_tab_state(self):
            # Create the state layout
            self.tab_state_layout = QtWidgets.QVBoxLayout(self.tab_state)

            # Create the state graph layout
            self.tab_state_graph_layout = pg.GraphicsLayoutWidget()
            self.tab_state_layout.addWidget(self.tab_state_graph_layout)
            
            # -------- All State plots -------- #
            __viewBox_all_states = pg.ViewBox(enableMenu=False)
            __viewBox_all_states.setXRange(-1, self.state_size, padding=0)
            __viewBox_all_states.setYRange(-1, 1, padding=0)
            __viewBox_all_states.setLimits(xMin=-1, xMax=self.state_size, yMin=-4, yMax=4)
            self.plot_all_states_item = pg.PlotItem(title="All States", viewBox=__viewBox_all_states)
            self.bar_graph_all_states           =   pg.BarGraphItem(x=range(self.state_size), height = np.zeros(self.state_size), width=0.5)
            self.plot_all_states_item.addItem(self.bar_graph_all_states)
            self.tab_state_graph_layout.addItem(self.plot_all_states_item, row=0, col=0, colspan=2, rowspan=1)

            # -------- Details State plots -------- #
            # Distance to Goal
            __viewBox_dtg = pg.ViewBox(enableMenu=False)
            __viewBox_dtg.setXRange(-1, 1, padding=0)
            __viewBox_dtg.setYRange(-1, 1, padding=0)
            __viewBox_dtg.setLimits(xMin=-1, xMax=1, yMin=-2, yMax=2)
            self.dtg_item = pg.PlotItem(title="Distance to Goal", viewBox=__viewBox_dtg)
            self.bar_graph_dtg                  =   pg.BarGraphItem(x=[0], height=[0], width=0.25)
            self.dtg_item.addItem(self.bar_graph_dtg)
            self.tab_state_graph_layout.addItem(self.dtg_item, row=1, col=0, colspan=1, rowspan=1)

            # Angle to Goal
            __viewBox_atg = pg.ViewBox(enableMenu=False)
            __viewBox_atg.setXRange(-3.14, 3.14, padding=0)
            __viewBox_atg.setYRange(-1, 1, padding=0)
            __viewBox_atg.setLimits(xMin=-3.14, xMax=3.14, yMin=-1, yMax=1)
            self.atg_item = pg.PlotItem(title="Angle to Goal", viewBox=__viewBox_atg)
            self.bar_graph_atg                  =   pg.BarGraphItem(x=[0], height=[0], width=0.25)
            self.bar_graph_atg.setRotation(-90)
            self.atg_item.addItem(self.bar_graph_atg)
            self.tab_state_graph_layout.addItem(self.atg_item, row=1, col=1, colspan=1, rowspan=1)
    
            # Theta
            __viewBox_theta = pg.ViewBox(enableMenu=False)
            __viewBox_theta.setXRange(-3.14, 3.14, padding=0)
            __viewBox_theta.setYRange(-1, 1, padding=0)
            __viewBox_theta.setLimits(xMin=-3.14, xMax=3.14, yMin=-1, yMax=1)
            self.theta_item = pg.PlotItem(title="Theta", viewBox=__viewBox_theta)
            self.bar_graph_theta                =   pg.BarGraphItem(x=[0], height=[0], width=0.25)
            self.bar_graph_theta.setRotation(-90)
            self.theta_item.addItem(self.bar_graph_theta)
            self.tab_state_graph_layout.addItem(self.theta_item, row=2, col=0, colspan=1, rowspan=1)

            # Angular Velocity
            __viewBox_angular = pg.ViewBox(enableMenu=False)
            __viewBox_angular.setXRange(-1, 1, padding=0)
            __viewBox_angular.setYRange(-1, 1, padding=0)
            __viewBox_angular.setLimits(xMin=-1, xMax=1, yMin=-1, yMax=1)
            self.angular_item = pg.PlotItem(title="Angular Velocity", viewBox=__viewBox_angular)
            self.bar_graph_angular              =   pg.BarGraphItem(x=[0], height=[0], width=0.25)
            self.bar_graph_angular.setRotation(-90)
            self.angular_item.addItem(self.bar_graph_angular)
            self.tab_state_graph_layout.addItem(self.angular_item, row=2, col=1, colspan=1, rowspan=1)

            # Last Action linear
            __viewBox_last_action_linear = pg.ViewBox(enableMenu=False)
            __viewBox_last_action_linear.setXRange(-1, 1, padding=0)
            __viewBox_last_action_linear.setYRange(-1, 1, padding=0)
            __viewBox_last_action_linear.setLimits(xMin=-1, xMax=1, yMin=-1, yMax=1)
            self.last_action_linear_item = pg.PlotItem(title="Last Action Linear", viewBox=__viewBox_last_action_linear)
            self.bar_graph_last_action_linear    =   pg.BarGraphItem(x=[0], height=[0], width=0.25)
            self.last_action_linear_item.addItem(self.bar_graph_last_action_linear)
            self.tab_state_graph_layout.addItem(self.last_action_linear_item, row=3, col=0, colspan=1, rowspan=1)

            # Last Action angular
            __viewBox_last_action_angular = pg.ViewBox(enableMenu=False)
            __viewBox_last_action_angular.setXRange(-1, 1, padding=0)
            __viewBox_last_action_angular.setYRange(-1, 1, padding=0)
            __viewBox_last_action_angular.setLimits(xMin=-1, xMax=1, yMin=-1, yMax=1)
            self.last_action_angular_item = pg.PlotItem(title="Last Action Angular", viewBox=__viewBox_last_action_angular)
            self.bar_graph_last_action_angular   =   pg.BarGraphItem(x=[0], height=[0], width=0.25)
            self.bar_graph_last_action_angular.setRotation(-90)
            self.last_action_angular_item.addItem(self.bar_graph_last_action_angular)
            self.tab_state_graph_layout.addItem(self.last_action_angular_item, row=3, col=1, colspan=1, rowspan=1)

            # ------- XY Lidar plane plots -------- #
            __viewBox_xy_lidar_plane = pg.ViewBox(enableMenu=False)
            __viewBox_xy_lidar_plane.setXRange(-1.5, 1.5, padding=0)
            __viewBox_xy_lidar_plane.setYRange(-1.5, 1.5, padding=0)
            __viewBox_xy_lidar_plane.setLimits(xMin=-3.0, xMax=3.0, yMin=-3.0, yMax=3.0)
            self.xy_lidar_plane_item = pg.PlotItem(title="XY Lidar Plane", viewBox=__viewBox_xy_lidar_plane)
            self.xy_lidar_plane_item.showGrid(x=True, y=True)
            self.tab_state_graph_layout.addItem(self.xy_lidar_plane_item, row=0, col=3, colspan=2, rowspan=2)
            
            # Position
            self.scatter_robot_xy_lidar         =   pg.ScatterPlotItem(x=[0], y=[0], size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 255))

            # Velocity
            self.arrow_robot_vxy_lidar           =   pg.PlotDataItem([0, 0], [0, 0], pen=pg.mkPen({'color': "#00FFFF", 'width': 2}))

            # Goal position
            self.scatter_goal_xy_lidar          =   pg.ScatterPlotItem(x=[0], y=[0], size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 0, 255))

            # Lidar 
            self.scatter_lidar                  =   pg.ScatterPlotItem(x=[0], y=[0], size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 255, 255))
            
            # Add the items to the XY lidar plane
            self.xy_lidar_plane_item.addItem(self.scatter_robot_xy_lidar)
            self.xy_lidar_plane_item.addItem(self.arrow_robot_vxy_lidar)
            self.xy_lidar_plane_item.addItem(self.scatter_goal_xy_lidar)
            self.xy_lidar_plane_item.addItem(self.scatter_lidar)

            # ------- XY Obstacle plane plots -------- #
            __viewBox_xy_obstacle_plane = pg.ViewBox(enableMenu=False)
            __viewBox_xy_obstacle_plane.setXRange(-1.5, 1.5, padding=0)
            __viewBox_xy_obstacle_plane.setYRange(-1.5, 1.5, padding=0)
            __viewBox_xy_obstacle_plane.setLimits(xMin=-3.0, xMax=3.0, yMin=-3.0, yMax=3.0)
            self.xy_obstacle_plane_item = pg.PlotItem(title="XY Obstacle Plane", viewBox=__viewBox_xy_obstacle_plane)
            self.xy_obstacle_plane_item.showGrid(x=True, y=True)
            self.tab_state_graph_layout.addItem(self.xy_obstacle_plane_item, row=2, col=3, colspan=2, rowspan=2)

            # Position
            self.scatter_robot_xy                =   pg.ScatterPlotItem(x=[0], y=[0], size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 255))
            self.scatter_obstacle_xy            =   pg.ScatterPlotItem(x=[0], y=[0], size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 255))

            # Velocity
            self.arrow_robot_vx                 =   pg.PlotDataItem([0, 0], [0, 0], pen=pg.mkPen({'color': "#00FF00", 'width': 2}))
            self.arrow_robot_vy                 =   pg.PlotDataItem([0, 0], [0, 0], pen=pg.mkPen({'color': "#00FF00", 'width': 2}))
            self.arrow_obstacle_vx              =   pg.PlotDataItem([0, 0], [0, 0], pen=pg.mkPen({'color': "#FF0000", 'width': 2}))
            self.arrow_obstacle_vy              =   pg.PlotDataItem([0, 0], [0, 0], pen=pg.mkPen({'color': "#FF0000", 'width': 2}))

            # Add the items to the XY obstacle plane
            self.xy_obstacle_plane_item.addItem(self.scatter_robot_xy)
            self.xy_obstacle_plane_item.addItem(self.arrow_robot_vx)
            self.xy_obstacle_plane_item.addItem(self.arrow_robot_vy)
            self.xy_obstacle_plane_item.addItem(self.scatter_obstacle_xy)
            self.xy_obstacle_plane_item.addItem(self.arrow_obstacle_vx)
            self.xy_obstacle_plane_item.addItem(self.arrow_obstacle_vy)

        def init_tab_actor(self):
            # Create the actor layout
            self.tab_actor_layout = QtWidgets.QVBoxLayout(self.tab_actor)

            # Create the actor graph layout
            self.tab_actor_graph_layout = pg.GraphicsLayoutWidget()
            self.tab_actor_layout.addWidget(self.tab_actor_graph_layout)

            # -------- Actor Hidden Layers plots -------- #
            self.actor_hidden_plot_items = []
            self.actor_hidden_bar_graphs = []
            self.actor_hidden_line_plots = []
            i = 0
            for hidden_size in self.hidden_sizes:
                plot_item = self.tab_actor_graph_layout.addPlot(title=f"Hidden layer {i}", row = i, col = 0, colspan=3)
                plot_item.setXRange(-1, hidden_size, padding=0)
                plot_item.setYRange(-0.2, 1.3, padding=0)

                bar_graph = pg.BarGraphItem(x=range(hidden_size), height=np.zeros(hidden_size), width=0.8)
                plot_item.addItem(bar_graph)

                line_plot = plot_item.plot(x=range(hidden_size), brush='r', symbol='x', symbolPen='r')
                line_plot.setPen(style=QtCore.Qt.NoPen)
                line_plot.setSymbolSize(5)

                self.actor_hidden_bar_graphs.append(bar_graph)
                self.actor_hidden_plot_items.append(plot_item)
                self.actor_hidden_line_plots.append(line_plot)
                i += 1

            # Actor Linear Action
            self.actor_action_linear_item = self.tab_actor_graph_layout.addPlot(title="Actor Linear Action"    , row=i, col=0, colspan=1, rowspan=1)
            self.actor_action_linear_item.setXRange(-1, 1, padding=0)
            self.actor_action_linear_item.setYRange(-1, 1, padding=0)
            self.bar_graph_action_linear         =   pg.BarGraphItem(x=[0], height=[0], width=0.5)
            self.actor_action_linear_item.addItem(self.bar_graph_action_linear)
            # Actor Angular Action
            self.actor_action_angular_item = self.tab_actor_graph_layout.addPlot(title="Actor Angular Action"  , row=i, col=1, colspan=1, rowspan=1)
            self.actor_action_angular_item.setXRange(-1, 1, padding=0)
            self.actor_action_angular_item.setYRange(-1, 1, padding=0)
            self.bar_graph_action_angular        =   pg.BarGraphItem(x=[0], height=[0], width=0.5)
            self.bar_graph_action_angular.setRotation(-90)
            self.actor_action_angular_item.addItem(self.bar_graph_action_angular)
            # Actor Reward
            self.actor_reward_item = self.tab_actor_graph_layout.addPlot(title="Actor Reward"                , row=i, col=2, colspan=1, rowspan=1)
            self.actor_reward_item.setXRange(-1, 1, padding=0)
            self.actor_reward_item.setYRange(-5000, 1000, padding=0)
            self.actor_bar_graph_reward          =   pg.BarGraphItem(x=[0], height=[0], width=0.5)
            self.actor_reward_item.addItem(self.actor_bar_graph_reward)

        def init_tab_critic(self):
            # Create the critic layout
            self.tab_critic_layout = QtWidgets.QVBoxLayout(self.tab_critic)

            # Create the critic graph layout
            self.tab_critic_graph_layout = pg.GraphicsLayoutWidget()
            self.tab_critic_layout.addWidget(self.tab_critic_graph_layout)

            # -------- Critic Hidden Layers plots -------- #
            self.critic_hidden_plot_items = []
            self.critic_hidden_bar_graphs = []
            self.critic_hidden_line_plots = []
            i = 0
            # Q1
            for hidden_size in self.hidden_sizes:
                plot_item = self.tab_critic_graph_layout.addPlot(title=f"Hidden layer {i}", row = i, col = 0, colspan=2)
                plot_item.setXRange(-1, hidden_size, padding=0)
                plot_item.setYRange(-0.2, 1.3, padding=0)

                bar_graph = pg.BarGraphItem(x=range(hidden_size), height=np.zeros(hidden_size), width=0.8)
                plot_item.addItem(bar_graph)

                line_plot = plot_item.plot(x=range(hidden_size), brush='r', symbol='x', symbolPen='r')
                line_plot.setPen(style=QtCore.Qt.NoPen)
                line_plot.setSymbolSize(5)

                self.critic_hidden_bar_graphs.append(bar_graph)
                self.critic_hidden_plot_items.append(plot_item)
                self.critic_hidden_line_plots.append(line_plot)
                i += 1
            # Q2
            j = 0
            for hidden_size in self.hidden_sizes:
                plot_item = self.tab_critic_graph_layout.addPlot(title=f"Hidden layer {i}", row = j, col = 2, colspan=2)
                plot_item.setXRange(-1, hidden_size, padding=0)
                plot_item.setYRange(-0.2, 1.3, padding=0)

                bar_graph = pg.BarGraphItem(x=range(hidden_size), height=np.zeros(hidden_size), width=0.8)
                plot_item.addItem(bar_graph)

                line_plot = plot_item.plot(x=range(hidden_size), brush='r', symbol='x', symbolPen='r')
                line_plot.setPen(style=QtCore.Qt.NoPen)
                line_plot.setSymbolSize(5)

                self.critic_hidden_bar_graphs.append(bar_graph)
                self.critic_hidden_plot_items.append(plot_item)
                self.critic_hidden_line_plots.append(line_plot)
                j += 1

            # Q1 Value
            self.critic_q1_item = self.tab_critic_graph_layout.addPlot(title="Critic Q1 Value"                , row=i, col=0, colspan=2)
            self.critic_q1_item.setXRange(-1, 1, padding=0)
            self.critic_q1_item.setYRange(-1000, 1000, padding=0)
            self.bar_graph_q1_value              =   pg.BarGraphItem(x=[0], height=[0], width=0.5)
            self.critic_q1_item.addItem(self.bar_graph_q1_value)
            # Q2 Value
            self.critic_q2_item = self.tab_critic_graph_layout.addPlot(title="Critic Q2 Value"                , row=i, col=2, colspan=2)
            self.critic_q2_item.setXRange(-1, 1, padding=0)
            self.critic_q2_item.setYRange(-1000, 1000, padding=0)
            self.bar_graph_q2_value              =   pg.BarGraphItem(x=[0], height=[0], width=0.5)
            self.critic_q2_item.addItem(self.bar_graph_q2_value)

        def prepare_data(self, tensor : torch.Tensor):
            return tensor.squeeze().detach().cpu()
        
        def tab_state_update(self, states):
            # States data
            states_data = self.prepare_data(states)

            # Update the states
            self.bar_graph_all_states.setOpts(height=states_data)

            lidar_data = states_data[0:NUM_SCAN_SAMPLES]
            state_others = states_data[NUM_SCAN_SAMPLES:]

            # Update the details states start from the end of the states_data
            dtg = state_others[0]
            atg = state_others[1]
            self.bar_graph_dtg.setOpts(height=[dtg])
            self.bar_graph_atg.setOpts(height=[atg])
            x = state_others[2]
            y = state_others[3]
            self.bar_graph_theta.setOpts(height=[state_others[4]])
            vx = state_others[5]
            vy = state_others[6]
            self.bar_graph_angular.setOpts(height=[state_others[7]])
            obs_x = state_others[8]
            obs_y = state_others[9]
            obs_vx = state_others[10]
            obs_vy = state_others[11]
            self.bar_graph_last_action_linear.setOpts(height=[state_others[12]])
            self.bar_graph_last_action_angular.setOpts(height=[state_others[13]])

            # ------- XY Lidar plane plots -------- #
            # Update the lidar
            _lidar_angles = np.linspace(0, 2*np.pi, NUM_SCAN_SAMPLES, endpoint=False)
            lidar_x = -lidar_data * np.cos(_lidar_angles)
            lidar_y = -lidar_data * np.sin(_lidar_angles)

            lidar_x += x
            lidar_y += y

            self.scatter_lidar.setData(x=lidar_x, y=lidar_y)

            # Update the XY position
            self.scatter_robot_xy_lidar.setData(x=[x], y=[y])

            # Calculate the new arrow end position based on velocity
            self.arrow_robot_vxy_lidar.setData([x, x + vx], [y, y + vy])

            # Update the goal position
            # Calculate the goal position based on the distance and angle to goal
            goal_x = x + dtg * np.cos(atg)
            goal_y = y + dtg * np.sin(atg)

            self.scatter_goal_xy_lidar.setData(x=[goal_x], y=[goal_y])

            # ------- XY Obstacle plane plots -------- #
            self.scatter_robot_xy.setData(x=[x], y=[y])
            self.scatter_obstacle_xy.setData(x=[obs_x], y=[obs_y])
            # Set the arrow position
            # Calculate the new arrow end position based on velocity
            self.arrow_robot_vx.setData([x, x + vx], [y, y])
            self.arrow_robot_vy.setData([x, x], [y, y + vy])
            self.arrow_obstacle_vx.setData([obs_x, obs_x + obs_vx], [obs_y, obs_y])
            self.arrow_obstacle_vy.setData([obs_x, obs_x], [obs_y, obs_y + obs_vy])

        def tab_actor_update(self, actions, hidden, biases):
            #Update the Actions
            actions = actions.detach().cpu().numpy().tolist()
            self.bar_graph_action_linear.setOpts(height=[actions[0]])
            self.bar_graph_action_angular.setOpts(height=[actions[1]])
            for i in range(len(hidden)):
                self.actor_hidden_bar_graphs[i].setOpts(height=self.prepare_data(hidden[i]))
            # pg.QtGui.QGuiApplication.processEvents()
            if self.actor_iteration % 50 == 0:
                for i in range(len(biases)):
                    self.actor_hidden_line_plots[i].setData(y=self.prepare_data(biases[i]))
            self.actor_iteration += 1

        def tab_critic_update(self, q_values, hidden, biases):
            # Update the Q values
            self.bar_graph_q1_value.setOpts(height=[q_values[0].detach().cpu()])
            self.bar_graph_q2_value.setOpts(height=[q_values[1].detach().cpu()])
            # Update hidden layers
            for i in range(len(hidden)):
                self.critic_hidden_bar_graphs[i].setOpts(height=self.prepare_data(hidden[i]))
            if self.critic_iteration % 50 == 0:
                for i in range(len(biases)):
                    self.critic_hidden_line_plots[i].setData(y=self.prepare_data(biases[i]))
            self.critic_iteration += 1

        # def update_layers(self, states, actions, hidden, biases):
        #     # States data
        #     states_data = self.prepare_data(states)

        #     # Update the states
        #     self.bar_graph_all_states.setOpts(height=states_data)

        #     lidar_data = states_data[0:NUM_SCAN_SAMPLES]
        #     state_others = states_data[NUM_SCAN_SAMPLES:]

        #     # Update the details states start from the end of the states_data
        #     dtg = state_others[0]
        #     atg = state_others[1]
        #     self.bar_graph_dtg.setOpts(height=[dtg])
        #     self.bar_graph_atg.setOpts(height=[atg])
        #     x = state_others[2]
        #     y = state_others[3]
        #     self.bar_graph_theta.setOpts(height=[state_others[4]])
        #     vx = state_others[5]
        #     vy = state_others[6]
        #     self.bar_graph_angular.setOpts(height=[state_others[7]])
        #     obs_x = state_others[8]
        #     obs_y = state_others[9]
        #     obs_vx = state_others[10]
        #     obs_vy = state_others[11]
        #     self.bar_graph_last_action_linear.setOpts(height=[state_others[12]])
        #     self.bar_graph_last_action_angular.setOpts(height=[state_others[13]])
        
        #     # Plot the XY position
        #     self.scatter_robot_xy.setData(x=[x], y=[y])
        #     self.scatter_obstacle_xy.setData(x=[obs_x], y=[obs_y])
        #     # Set the arrow position
        #     # Calculate the new arrow end position based on velocity
        #     self.arrow_robot_vx.setData([x, x + vx], [y, y])
        #     self.arrow_robot_vy.setData([x, x], [y, y + vy])
        #     self.arrow_obstacle_vx.setData([obs_x, obs_x + obs_vx], [obs_y, obs_y])
        #     self.arrow_obstacle_vy.setData([obs_x, obs_x], [obs_y, obs_y + obs_vy])

        #     # Update the lidar
        #     _lidar_angles = np.linspace(0, 2*np.pi, NUM_SCAN_SAMPLES, endpoint=False)
        #     lidar_x = -lidar_data * np.cos(_lidar_angles)
        #     lidar_y = -lidar_data * np.sin(_lidar_angles)

        #     lidar_x += x
        #     lidar_y += y

        #     self.scatter_lidar.setData(x=lidar_x, y=lidar_y)

        #     # Update the goal position
        #     # Calculate the goal position based on the distance and angle to goal
        #     goal_x = x + dtg * np.cos(atg)
        #     goal_y = y + dtg * np.sin(atg)

        #     self.scatter_goal_xy.setData(x=[goal_x], y=[goal_y])

        #     #Update the Actions
        #     actions = actions.detach().cpu().numpy().tolist()
        #     self.bar_graph_action_linear.setOpts(height=[actions[0]])
        #     self.bar_graph_action_angular.setOpts(height=[actions[1]])
        #     for i in range(len(hidden)):
        #         self.actor_hidden_bar_graphs[i].setOpts(height=self.prepare_data(hidden[i]))
        #     pg.QtGui.QGuiApplication.processEvents()
        #     if self.iteration % 100 == 0:
        #         self.update_bias(biases)
        #     self.iteration += 1

        # def update_bias(self, biases):
        #     for i in range(len(biases)):
        #         self.actor_hidden_line_plots[i].setData(y=self.prepare_data(biases[i]))

        def update_reward(self, acc_reward):
            self.actor_bar_graph_reward.setOpts(height=[acc_reward])
            if acc_reward > 0:
                self.actor_bar_graph_reward.setOpts(brush='g')
            else:
                self.actor_bar_graph_reward.setOpts(brush='r')