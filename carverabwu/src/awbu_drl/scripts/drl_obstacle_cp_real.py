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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
import random
import cp_utils as utils 
import copy
import numpy as np

from awbu_interfaces.msg import Obstacle
from awbu_interfaces.srv import ScoreStep

from env_utils import get_simulation_speed, read_stage, bcolors

from settings.constparams import THRESHOLD_COLLISION, TOPIC_ODOM, TOPIC_SCAN, TOPIC_CLOCK
ROBOT_WIDTH = 0.3

sim_speed = get_simulation_speed(read_stage())

TIME_STEP  = 1/10
ALPHA = 0.5 # for CP
MIN_DISTANCE = 0.05 # for tracking Missing object using KALMAN
_RADIUS = 1 # object should have low radius

class Object:
    def __init__(self, center = (0., 0.), radius = 1.):
        self.ID = None
        self.position = None
        self.velocity = (0. , 0.)
        self.center = center
        self.radius = radius
        self.type = None
        self.time = 0.
        self.time_step = TIME_STEP
        self.Predicted_KF = ( 0. , 0. , 0. ,0)
        
        dt = self.time_step

        # Kalman Filter

        self.F = np.array( # state transition matrix
            [[1, 0, dt, 0], # x = x + vx * dt
            [0, 1,  0, dt], # y = y + vy * dt
            [0, 0,  1, 0], # vx = vx
            [0, 0,  0, 1] # vy = vy
                      ])
        
        self.G = np.array( # control input matrix
            [[0.5 * dt**2, 0], # x = 0.5 * ax * dt^2
            [0, 0.5 * dt**2], # y = 0.5 * ay * dt^2
            [dt, 0], # vx = ax * dt
            [0, dt] # vy = ay * dt
                      ])
        
        self.Q = np.array( # process noise covariance matrix
            [[0.1, 0.00, 0.00, 0.00], 
            [0.00, 0.1, 0.00, 0.00],
            [0.00, 0.00, 0.1, 0.00],
            [0.00, 0.00, 0.00, 0.1]
                      ])
        
        self.H = np.array( # observation matrix
            [[1, 0, 0 ,0], # x = x
            [0, 1, 0 ,0]] # y = y
                     )
        
        self.R = np.array([# observation noise covariance matrix
                    [0.001 , 0.0],
                    [0.0 , 0.001]])
        
        self.P = np.array([ # initial estimate covariance matrix
            [0.1, 0.0, 0.0, 0.0],
            [0.0, 0.1, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0.0],
            [0.0, 0.0, 0.0, 0.1]
                      ])
                
        self.x = np.array([center[0], center[1], 0., 0.]).reshape(4,1)

        self.kf = KalmanFilter(self.F, self.G, self.Q, self.H, self.R, self.P, self.x)
        
        self._missing_frame = 0
        self.max_missing_frame = 2 *30
        self.CP = 0.0

    def predict_kf(self):
        # [x, y, vx, vy]
        x = self.kf.get_state_estimate()
        return [x[0,0], x[1,0], x[2,0], x[3,0]]
    
class KalmanFilter:
    def __init__(self, 
    #-- State Model--#
    F, # State Transition Matrix
    G, # Control Input Matrix
    Q, # Process Noise Covariance
    #-- Measurement Model--#
    H, # Measurement Matrix
    R, # Measurement Noise Covariance
    #-- Initial State--#
    P, # Initial Estimate Covariance
    x  # Initial State Estimate
    ):
        # initialize the state model
        self.F = F
        self.G = G
        self.Q = Q
        self.H = H
        self.R = R
        self.P = P
        self.x = x

        # initialize the state estimate
        self.x_hat = self.x
        self.P_hat = self.P

        self.x_hat_prev = self.x_hat
        self.P_hat_prev = self.P_hat

        # Convert numpy arrays to matrices
        self.F = np.matrix(self.F)
        self.G = np.matrix(self.G)
        self.Q = np.matrix(self.Q)
        self.H = np.matrix(self.H)
        self.R = np.matrix(self.R)
        self.P = np.matrix(self.P)
        self.x = np.matrix(self.x)
        self.x_hat = np.matrix(self.x_hat)
        self.P_hat = np.matrix(self.P_hat)
        self.x_hat_prev = np.matrix(self.x_hat_prev)
        self.P_hat_prev = np.matrix(self.P_hat_prev)
    
    def update(self, 
        z, # Measurement
        u = np.matrix([[0], [0]]), # Control Input
    ):
        # Predict state estimate
        # F*Xt-1 + G*Ut
        self.x_hat = self.F * self.x_hat_prev + self.G * u

        # Predict estimate covariance
        # F*Pt-1*F^T + Q
        self.P_hat = self.F * self.P_hat_prev * self.F.T + self.Q

        # Calculate Kalman Gain
        # P*H^T*(H*P*H^T + R)^-1
        K = self.P_hat * self.H.T * np.linalg.inv(self.H * self.P_hat * self.H.T + self.R)

        # Update state estimate
        # Xt = Xt + K*(Zt - H*Xt)
        self.x_hat = self.x_hat + K * (z - self.H * self.x_hat)

        # Update estimate covariance
        # (I - K*H)*P
        self.P_hat = (np.eye(self.P.shape[0]) - K * self.H) * self.P_hat

        # Update previous state estimate and covariance
        self.x_hat_prev = self.x_hat
        self.P_hat_prev = self.P_hat

        return self.x_hat

    
    def get_state_estimate(self):
        return self.x_hat

class ObstacleCPHandler(Node):

    def __init__(self):
        super().__init__('ObstacleCPHandler')  

        # QoS profile
        qos_clock = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        qos_default = QoSProfile(
            depth=10,
        )

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,
            TOPIC_ODOM,
            self.get_odometry,
            qos_default)

        self.sub_scan = self.create_subscription(
            LaserScan,
            TOPIC_SCAN,
            self.scan_callback,
            qos_default)
        self.sub_clock  = self.create_subscription(
            Clock, 
            TOPIC_CLOCK,
            self.clock_callback, 
            qos_profile=qos_clock)
        
        # Publishers
        self._visual_publisher_raw = self.create_publisher(
            MarkerArray,
            'ObstacleVis_raw', 
            qos_default)
        self._visual_publisher_cp = self.create_publisher(
            MarkerArray, 
            'ObstacleVis_cp', 
            qos_default)
        self.CP_publisher = self.create_publisher(
            Obstacle, 
            '/abwubot/obstacleCP', 
            qos_default)
        
        # Service server
        self.reset_simulation_service = self.create_service(
            Empty, 
            'reset_obstacle_cp', 
            self.cp_reset_srv_callback)

        self.get_k_t_score = self.create_service(
            ScoreStep, 
            'score_step_comm', 
            self.get_time_stepscore_srv_callback)
        
        # Initialize variables
        self.ID = 0
        self.Current_group = {}
        self.Previous_group = {}
        self._Start = True
        self.total_current_object = 0
        self.total_previous_object = 0

        # Parameters
        self._dth = 0.001
        self._max_cluster_size = 360
        self._euclidean_distance = 0.25

        # Color track
        self.color_track = {}

        # Score parameters
        self.k_time = 0.0
        self.m_time = 0.0
        self.time_step = 0.0

        # Create a timer with a period of 1 second (1000 milliseconds)
        self.timer = self.create_timer(TIME_STEP,  self.timer_callback)

        self.scan = None
        self.time_sec = None
        self.position = None

        self.past_time = 0.0

    def get_time_stepscore_srv_callback(self, request: ScoreStep.Request, response: ScoreStep.Response):
        self.get_logger().info('Get Score')
        response.k_time = float(self.k_time)
        response.m_time = float(self.m_time)
        response.total_time = float(self.time_step)
        return response

    def _obstacle_pubish(self,_id,center_x,center_y,velocity_x,velocity_y,CP):

        self._OBSTACLE = Obstacle()

        self._OBSTACLE.id = _id
        self._OBSTACLE.pose_x = center_x
        self._OBSTACLE.pose_y = center_y 
        self._OBSTACLE.velocity_x = velocity_x
        self._OBSTACLE.velocity_y = velocity_y
        self._OBSTACLE.cp = CP

        self.CP_publisher.publish(self._OBSTACLE)
        
    def clock_callback(self, msg: Clock):
        # Get current time
        seconds= msg.clock.sec 
        nanoseconds = msg.clock.nanosec 
        self.time_sec = seconds + nanoseconds / 1e9
        # print("TIME" , self.time_sec)

    def cp_reset_srv_callback(self, request, response):
        self.ID = 0
        self.Current_group = {}
        self.Previous_group = {}
        self._Start = True
        self.total_current_object = 0
        self.total_previous_object = 0
        self.color_track = {}

        self.k_time = 0.0
        self.m_time = 0.0
        self.time_step = 0.0

        self.clear_visual()

        self.past_time = self.time_sec

        response = Empty.Response()

        return response
    
    def clear_visual(self):

        marker = Marker()
        marker.header.frame_id = 'base_link'  # or the appropriate frame id for your use case
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ''
        marker.id = 0
        marker.type = Marker.CUBE  # The type can be any valid marker type
        marker.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self._visual_publisher_cp.publish(marker_array)
        self._visual_publisher_raw.publish(marker_array)
    
    def scan_callback(self ,msg: LaserScan):
        self.scan = msg
        self.max_scan = msg.range_max
        self.min_scan = msg.range_min

    def get_object_group(self, msg: LaserScan):

        # Get scan data
        scan = msg
        # Get robot position
        x_robot = self.position.x
        y_robot = self.position.y

        # Get robot orientation
        orientation_list = [self.orientation.x , self.orientation.y , self.orientation.z , self.orientation.w]
        _ , _ , yaw = euler_from_quaternion(orientation_list)
        robot_pose = x_robot , y_robot , yaw
        
        # Cluster the scan data
        group_list = utils.Clustering(scan)    

        object_list = []

        for group in group_list:
            # Transform local to global
            tranform_lambda = lambda x : utils.transform_local_to_global(robot_pose, x)

            locals_group = list(map(tranform_lambda, group))

            # Classify object
            _type, center, radius = utils.classify_object(locals_group)

            if _type == "Obstacle":
                # print(f'Center: {center}, Radius: {radius}')

                # Object 
                _obj = {
                    'type': _type,
                    'center': center,
                    'radius': radius,
                    'points': locals_group
                }

                object_list.append(_obj)

        # Update total current object
        self.total_current_object = len(object_list)

        # number_obstacle = len([1 for x in object_list if x['type'] == 'Obstacle'])

        # print(f'Total current object: {self.total_current_object} number of obstacle: {number_obstacle}')

        return object_list
    
    def object_tracking_process(self, object_list):
        
        # self.Current_group = {
        #     ID : Object
        # }

        if self._Start:
            # assign group by ID
            for __obj in object_list:

                group = Object(center=__obj['center'], radius=__obj['radius'])
                group.ID = self.ID 
                group.position = __obj['points']
                group.velocity = (0. , 0.) 
                group.type = __obj['type'] 
                group.time = self.time_sec

                # Add object to the current group
                self.Current_group[self.ID] = group

                self.ID += 1

            # Set start to False
            self._Start = False

            # Update previous object
            self.total_previous_object = self.total_current_object
            self.Previous_group = copy.deepcopy(self.Current_group)

        else :
            
            if self.Previous_group != {} : 

                #------------------ Handle object number change ------------------#
                if self.total_current_object > self.total_previous_object:
                    # Add new object
                    for obj in object_list:
                        
                        obj_center = obj['center']

                        # Check the distance between the new object and the previous object
                        distances = [np.linalg.norm(np.array(obj_center) - np.array(x.center)) for x in self.Previous_group.values()]

                        # If the distance is less than the threshold, then it is the same object
                        if min(distances) < 1.0:
                            continue

                        # Create new object
                        group = Object(center=obj_center, radius=obj['radius'])
                        group.ID = self.ID
                        group.position = obj['points']
                        group.velocity = (0. , 0.)
                        group.type = obj['type']
                        group.time = self.time_sec

                        # Add object to the current group
                        self.Current_group[self.ID] = group

                        self.ID += 1

                #------------------ Handle object association ------------------#

                # check association only obstacle
                center_obstable_group = [x['center'] for x in object_list]

                # Association return (ID , index)
                associations = utils.association(self.Previous_group, center_obstable_group)

                for ID , index in associations : 
                    # Get object from object list
                    __obj = object_list[index]
                    # Get previous object
                    pre_obj = self.Previous_group[ID]

                    # Get the current object
                    current_obj = self.Current_group[ID]

                    # pre_center = np.array(pre_obj.center)
                    current_center = np.array(__obj['center'])

                    dt = self.time_sec - pre_obj.time

                    if dt != 0.0 :

                        observation = np.array([current_center[0], current_center[1]]).reshape(2,1)
                    
                        # print(f'observation: \n{observation}')
                        # Update Kalman filter
                        state_estimate = pre_obj.kf.update(observation)

                        # print(f'X: {state_estimate[0,0]:.2f} Y: {state_estimate[1,0]:.2f} VX: {state_estimate[2,0]:.2f} VY: {state_estimate[3,0]:.2f}')

                        # Update object
                        current_obj.position = __obj['points'] 
                        current_obj.velocity = (state_estimate[2,0], state_estimate[3,0])
                        current_obj.Predicted_KF = (state_estimate[0,0], state_estimate[1,0], state_estimate[2,0], state_estimate[3,0])
                        current_obj.center = current_center
                        current_obj.radius = __obj['radius']
                        current_obj.kf = copy.deepcopy(pre_obj.kf)
                        current_obj._missing_frame = 0

                    # If velocity is large, then it is not a valid object
                    if np.linalg.norm(current_obj.velocity) > 1.0:
                        # self.get_logger().info(bcolors.FAIL + f'Velocity is too large: {np.linalg.norm(current_obj.velocity)}' + bcolors.ENDC)
                        self.Current_group.pop(ID)

                # #------------------ Handle missing object ------------------#
                # missing_ID = [ID for ID in self.Previous_group if ID not in [x[0] for x in associations]]
                # # self.get_logger().info(f'All ID: { [x for x in self.Previous_group.keys()]}, len: {len(self.Previous_group.keys())}')
                # # self.get_logger().info(f'Missing ID: {missing_ID}')
                # # if the object is missing add to the missing frame
                # for ID in missing_ID:
                #     # Get current object
                #     current_obj = self.Current_group[ID]

                #     # Add missing frame
                #     current_obj._missing_frame += 1

                #     # print(f'ID: {ID} missing frame: {current_obj._missing_frame}')

                #     # If the missing frame is greater than the max missing frame
                #     if current_obj._missing_frame > current_obj.max_missing_frame:
                        
                #         self.get_logger().info(bcolors.FAIL + f'ID: {ID} is missing' + bcolors.ENDC)

                #         # Remove the object
                #         # self.Current_group.pop(ID)

                    
                self.Previous_group = copy.deepcopy(self.Current_group)
                self.total_previous_object = self.total_current_object

                self.visualize_grouped_points(self.Current_group)

            else:
                # self.get_logger().info(bcolors.FAIL + 'Previous group is empty' + bcolors.ENDC)
                self._Start = True


    def timer_callback(self):
        # Return if any of the required data is missing
        if self.scan == None or self.time_sec == None or self.position == None: return

        object_list = self.get_object_group(self.scan)

        self.object_tracking_process(object_list)

        # Print all object velocity
        # for ID, obj in self.Current_group.items():
        #     if obj.type == 'Obstacle':
        #         self.get_logger().info(bcolors.OKGREEN + f'ID: {ID} Velocity: {obj.velocity}' + bcolors.ENDC)

        # Calculate CP
        x_robot = self.position.x
        y_robot = self.position.y

        ID_LIST = [] 
        CENTER_X = []
        CENTER_Y = []
        VELOCITY_X = []
        VELOCITY_Y = []
        CP_LIST = []
        DIST_ARRAY = []
        Pc_ttc_ARRAY = []

        if self._Start == False and self.Previous_group != {}:
            # print(self.Current_group)
            for ID in self.Current_group:
                if self.Current_group[ID].type == "Obstacle" and ID in self.Previous_group.keys():
                    # print("============== ID : {} =================" . format(ID))  
                    # print()
                    # distance = np.array(self.Current_group[ID].center) - np.array(self.Previous_group[ID].center)
                    dt = self.time_sec - self.Previous_group[ID].time

                    # if dt < 1e-10 : continue
                    # center_kf = [self.Current_group[ID].Predicted_KF [0] , self.Current_group[ID].Predicted_KF[1]]
                    # velocity_kf = [self.Current_group[ID].Predicted_KF [2] , self.Current_group[ID].Predicted_KF[3]]

                    # print("Distance :" , distance)
                    # print("dt :",dt)
                    # print("Center :" , self.Current_group[ID].center)
                    # print('velo :' , self.Current_group[ID].velocity)
                    # print("Predicted KF :" , self.Current_group[ID].Predicted_KF)

                    #### Calculate CP 
                    n = len(self.Current_group[ID].position)
                    points = self.Current_group[ID].position
                    
                    pos = np.array([x_robot , y_robot])
                    dist = []
                    for i in range(n):
                        dist.append(np.sqrt((pos[0] - points[i][0])**2 + (pos[1] - points[i][1])**2))

                    Dist_o = min(dist)

                    DIST_ARRAY.append(Dist_o)

                    # Calculate Pc_ttc
                    Vr = np.array([self.linear_twist.x , self.linear_twist.y])
                    Vo = np.array(self.Current_group[ID].velocity)
                    Vr_prime = Vr - Vo
                    t = Dist_o / np.sqrt(Vr_prime[0]**2 + Vr_prime[1]**2)

                    Pc_ttc = min([ 1, dt / t])

                    Pc_ttc_ARRAY.append(Pc_ttc)

                    # Calculate Pc_dto
                    Imax = self.max_scan
                    Imin = self.min_scan 

                    Pc_dto = (Imax - Dist_o) / (Imax - Imin)

                    
                    CP = ALPHA * Pc_ttc + (1-ALPHA) * Pc_dto

                    self.Current_group[ID].CP = CP

                    # print("Pc_ttc : " ,Pc_ttc)
                    # print("Pc_dto : " ,Pc_dto)
                    # print("Collision Probability (CP) : " , CP)
                    
                    if not np.isnan(self.Current_group[ID].velocity[0]) and not np.isnan(self.Current_group[ID].velocity[1]) :
                        if not np.isinf(self.Current_group[ID].velocity[0]) and not np.isinf(self.Current_group[ID].velocity[1]) :
                            ID_LIST.append(ID)
                            CENTER_X.append(self.Current_group[ID].center[0])
                            CENTER_Y.append(self.Current_group[ID].center[1])
                            VELOCITY_X.append(self.Current_group[ID].velocity[0])
                            VELOCITY_Y.append(self.Current_group[ID].velocity[1])
                            CP_LIST.append(CP)

            if DIST_ARRAY != [] and Pc_ttc_ARRAY != []:
                if min(DIST_ARRAY) - THRESHOLD_COLLISION < 0.787 * ROBOT_WIDTH : ## 0.5 + 0.5

                    self.k_time += 1

                if max(Pc_ttc_ARRAY) > 0.4 :

                    self.m_time += 1

        # Update time step
        self.time_step += 1        

        self._obstacle_pubish(ID_LIST , CENTER_X , CENTER_Y , VELOCITY_X , VELOCITY_Y , CP_LIST)

    def get_odometry(self, odom):
        self.position = odom.pose.pose.position
        self.orientation = odom.pose.pose.orientation
        self.linear_twist = odom.twist.twist.linear
        self.angular_twist = odom.twist.twist.angular


    def visualize_grouped_points(self, point_clusters): ### point_clusters  -> self.Current_group

        marker_array = MarkerArray()
        cg = 0  # Counter for marker IDs
        # print("len color Track :" , len(self.color_track))

        temp = copy.deepcopy(point_clusters)

        x_robot = self.position.x
        y_robot = self.position.y

        orientation_list = [self.orientation.x , self.orientation.y , self.orientation.z , self.orientation.w]
        _ , _ , yaw = euler_from_quaternion(orientation_list)

        robot_pose = x_robot , y_robot , yaw

        for ID in point_clusters: 
            if temp[ID].type == "Obstacle" :
                position = []
                for pos in temp[ID].position :
                    position.append(utils.transform_global_to_local(robot_pose , pos))

                temp[ID].position = position
                temp[ID].velocity = robot_pose,point_clusters[ID].velocity
                # center_kf = [point_clusters[ID].Predicted_KF [0] , point_clusters[ID].Predicted_KF[1]]
                center = point_clusters[ID].center
                temp[ID].center = utils.transform_global_to_local(robot_pose ,center)

            elif temp[ID].type == "WALL" :
                position = []
                for pos in temp[ID].position :
                    position.append(utils.transform_global_to_local(robot_pose , pos))
                temp[ID].position = position
                temp[ID].velocity = None
                temp[ID].center = None

        for i in temp : 
            if i not in self.color_track : 
                self.color_track[i] = [random.random() , random.random() ,random.random()]
        # print(self.color_track)

        for ID in temp:
            gpoints = Marker()
            gpoints.header.frame_id = 'base_link'  # Replace 'base_link' with your desired frame
            gpoints.header.stamp = self.get_clock().now().to_msg()
            gpoints.ns = 'clustered_points'
            gpoints.action = Marker.ADD
            gpoints.pose.orientation.w = 1.0
            gpoints.type = Marker.POINTS
            gpoints.scale.x = 0.04
            gpoints.scale.y = 0.04

            if temp[ID].type == "WALL" or temp[ID].type  == None:
                color = [1. , 1., 1.]

            else :
                color = self.color_track[ID]
        
    
            
            gpoints.color.r = color[0]
            gpoints.color.g = color[1]
            gpoints.color.b = color[2]
            gpoints.color.a = 1.0

            for point in temp[ID].position:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                gpoints.points.append(p)

            if temp[ID].type == "Obstacle" : 
                center, radius = temp[ID].center , temp[ID].radius
                p = Point()
                p.x = center[0]
                p.y = center[1]
                p.z = 0.0
                gpoints.points.append(p)

            gpoints.id = cg
            cg += 1




            marker_array.markers.append(gpoints)
        
        self._visual_publisher_raw.publish(marker_array)

        a = 0
        marker_array2 = MarkerArray()
    
        for ID in temp: 
            if temp[ID].type == "Obstacle" : 
                marker = Marker()

                center = temp[ID].center 
                radius = temp[ID].radius
                
                scaled_radius = radius / 0.5

                marker.header.frame_id = "base_link"
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                marker.scale.x = 0.1  # Cylinder diameter
                marker.scale.y = 0.1  # Cylinder diameter
                marker.scale.z = 0.1  # Cylinder height

                color = self.color_track[ID]

                marker.color.a = 1.0  # Fully opaque
                marker.color.r = color[0]  # Red color
                marker.color.g = color[1]  # Red color
                marker.color.b = color[2]  # Red color

                marker.pose.position.x = center[0] # Position (vary for each marker)
                marker.pose.position.y = center[1] # Position (vary for each marker)
                marker.pose.position.z = 0.0  # Position
                marker.pose.orientation.w = 1.0  # No rotation
                marker.id = a
                a += 1
                marker_array2.markers.append(marker)

                marker = Marker()

                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "text_marker"
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD

                marker.pose.position.x = center[0]
                marker.pose.position.y = center[1] + 2
                marker.pose.position.z = 1.0

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.z = 0.5  # Height of the text in meters
                marker.color.a = 1.0  # Alpha (0.0 is fully transparent, 1.0 is fully opaque)
                marker.color.r = 1.0  # Red
                marker.color.g = 1.0  # Green
                marker.color.b = 1.0  # Blue

                # marker.text = "CP : {}".format(temp[ID].CP)
                marker.text = f"ID: {ID} CP: {temp[ID].CP:.2f}"
                marker.id = a
                a += 1
                marker_array2.markers.append(marker)




        self._visual_publisher_cp.publish(marker_array2)




def main(args=None):
    rclpy.init(args=args)

    node = ObstacleCPHandler()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
