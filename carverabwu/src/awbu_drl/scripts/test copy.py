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
            self.hidden_sizes = [hidden_size, hidden_size]

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
            # self.init_tab_critic()
            
            self.iteration = 0

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
            self.arrow_robot_vxy_lidar           =   pg.PlotDataItem([0, 0], [0, 0], pen=pg.mkPen({'color': "#00FF00", 'width': 2}))

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

            # Goal position
            self.scatter_goal_xy                =   pg.ScatterPlotItem(x=[0], y=[0], size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 0, 255))

            # Add the items to the XY obstacle plane
            self.xy_obstacle_plane_item.addItem(self.scatter_robot_xy)
            self.xy_obstacle_plane_item.addItem(self.arrow_robot_vx)
            self.xy_obstacle_plane_item.addItem(self.arrow_robot_vy)
            self.xy_obstacle_plane_item.addItem(self.scatter_obstacle_xy)
            self.xy_obstacle_plane_item.addItem(self.arrow_obstacle_vx)
            self.xy_obstacle_plane_item.addItem(self.arrow_obstacle_vy)
            self.xy_obstacle_plane_item.addItem(self.scatter_goal_xy)

        
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
            self.bar_graph_action_linear         =   pg.BarGraphItem(x=[1], height=[0], width=0.5)
            self.actor_action_linear_item.addItem(self.bar_graph_action_linear)
            # Actor Angular Action
            self.actor_action_angular_item = self.tab_actor_graph_layout.addPlot(title="Actor Angular Action"  , row=i, col=1, colspan=1, rowspan=1)
            self.actor_action_angular_item.setXRange(-1, 1, padding=0)
            self.actor_action_angular_item.setYRange(-1, 1, padding=0)
            self.bar_graph_action_angular        =   pg.BarGraphItem(x=[1], height=[0], width=0.5)
            self.bar_graph_action_angular.setRotation(-90)
            self.actor_action_angular_item.addItem(self.bar_graph_action_angular)
            # Actor Reward
            self.actor_reward_item = self.tab_actor_graph_layout.addPlot(title="Actor Reward"                , row=i, col=2, colspan=1, rowspan=1)
            self.actor_reward_item.setXRange(-1, 1, padding=0)
            self.actor_reward_item.setYRange(-5000, 1000, padding=0)
            self.actor_bar_graph_reward          =   pg.BarGraphItem(x=[1], height=[0], width=0.5)
            self.actor_reward_item.addItem(self.actor_bar_graph_reward)


        def prepare_data(self, tensor : torch.Tensor):
            return tensor.squeeze().flip(0).detach().cpu()

        def update_layers(self, states, actions, hidden, biases):
            # States data
            states_data = self.prepare_data(states)

            # Update the states
            self.bar_graph_all_states.setOpts(height=states_data)

            START_IDX = self.state_size - NUM_SCAN_SAMPLES - 1

            # Update the details states start from the end of the states_data
            self.bar_graph_dtg.setOpts(height=[states_data[START_IDX]])
            self.bar_graph_atg.setOpts(height=[states_data[START_IDX-1]])
            self.bar_graph_theta.setOpts(height=[states_data[START_IDX-4]])
            x = states_data[START_IDX-2]
            y = states_data[START_IDX-3]
            vx = states_data[START_IDX-5]
            vy = states_data[START_IDX-6]
            self.bar_graph_angular.setOpts(height=[states_data[START_IDX-7]])
            obs_x = states_data[START_IDX-8]
            obs_y = states_data[START_IDX-9]
            obs_vx = states_data[START_IDX-10]
            obs_vy = states_data[START_IDX-11]
            self.bar_graph_last_action_linear.setOpts(height=[states_data[START_IDX-12]])
            self.bar_graph_last_action_angular.setOpts(height=[states_data[START_IDX-13]])
        
            # Plot the XY position
            self.scatter_robot_xy.setData(x=[x], y=[y])
            self.scatter_obstacle_xy.setData(x=[obs_x], y=[obs_y])
            # Set the arrow position
            # Calculate the new arrow end position based on velocity
            self.arrow_robot_vx.setData([x, x + vx], [y, y])
            self.arrow_robot_vy.setData([x, x], [y, y + vy])
            self.arrow_obstacle_vx.setData([obs_x, obs_x + obs_vx], [obs_y, obs_y])
            self.arrow_obstacle_vy.setData([obs_x, obs_x], [obs_y, obs_y + obs_vy])

            #Update the Actions
            actions = actions.detach().cpu().numpy().tolist()
            self.bar_graph_action_linear.setOpts(height=[actions[0]])
            self.bar_graph_action_angular.setOpts(height=[actions[1]])
            for i in range(len(hidden)):
                self.actor_hidden_bar_graphs[i].setOpts(height=self.prepare_data(hidden[i]))
            pg.QtGui.QGuiApplication.processEvents()
            if self.iteration % 100 == 0:
                self.update_bias(biases)
            self.iteration += 1

        def update_bias(self, biases):
            for i in range(len(biases)):
                self.actor_hidden_line_plots[i].setData(y=self.prepare_data(biases[i]))

        def update_reward(self, acc_reward):
            self.actor_bar_graph_reward.setOpts(height=[acc_reward])
            if acc_reward > 0:
                self.actor_bar_graph_reward.setOpts(brush='g')
            else:
                self.actor_bar_graph_reward.setOpts(brush='r')

def main(): 
    app = QtWidgets.QApplication([])
    stae_size = 150
    hidden_size = 100
    visualizer = DrlVisual(stae_size, hidden_size)
    for i in range(1000):
        states = torch.randn(150)
        actions = torch.randn(2)
        hidden = [torch.randn(hidden_size), torch.randn(hidden_size)]
        biases = [torch.randn(hidden_size), torch.randn(hidden_size)]
        visualizer.update_layers(states, actions, hidden, biases)
        time.sleep(0.1)

    app.exec_()

    
if __name__ == "__main__":
    main()