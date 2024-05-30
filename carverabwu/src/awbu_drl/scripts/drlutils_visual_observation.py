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

from settings.constparams import ENABLE_VISUAL, NUM_SCAN_SAMPLES

if ENABLE_VISUAL:
    from PyQt5 import QtWidgets, QtCore
    import pyqtgraph as pg
    import numpy as np
    import time

    import torch

    pg.setConfigOptions(antialias=False)

    class VisualObservation(QtWidgets.QMainWindow):
        def __init__(self, state_size):
            super().__init__(None)
            # Set the window 
            self.show()
            self.resize(900, 600)

            self.state_size = state_size

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

            # Tab Initialization
            self.init_tab_state()
        

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
            self.tab_state_graph_layout.addItem(self.plot_all_states_item, row=0, col=0, colspan=4, rowspan=1)


            # ------- XY Lidar plane plots -------- #
            __viewBox_xy_lidar_plane = pg.ViewBox(enableMenu=False)
            __viewBox_xy_lidar_plane.setXRange(-1.5, 1.5, padding=0)
            __viewBox_xy_lidar_plane.setYRange(-1.5, 1.5, padding=0)
            __viewBox_xy_lidar_plane.setLimits(xMin=-3.0, xMax=3.0, yMin=-3.0, yMax=3.0)
            self.xy_lidar_plane_item = pg.PlotItem(title="XY Lidar Plane", viewBox=__viewBox_xy_lidar_plane)
            self.xy_lidar_plane_item.showGrid(x=True, y=True)
            self.tab_state_graph_layout.addItem(self.xy_lidar_plane_item, row=1, col=0, colspan=2, rowspan=2)
            
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
            self.tab_state_graph_layout.addItem(self.xy_obstacle_plane_item, row=1, col=2, colspan=2, rowspan=2)

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

        def prepare_data(self, tensor : torch.Tensor):
            try:
                return tensor.squeeze().detach().cpu()
            except:
                return np.array(tensor)
        
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
            x = state_others[2]
            y = state_others[3]
            vx = state_others[5]
            vy = state_others[6]
            obs_x = state_others[8]
            obs_y = state_others[9]
            obs_vx = state_others[10]
            obs_vy = state_others[11]


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