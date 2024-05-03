#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy
import random
import utils 
import copy
import numpy as np

TIME_STEP  = 1/5 #hz - > second

class Object:
    def __init__(self) -> None:
        self.position = None
        self.velocity = (0. , 0.)
        self.center = (0. , 0.)
        self.radius = None
        self.type = None
        self.time = 0.
        self.time_step = TIME_STEP
        self.Predicted_KF = ( 0. , 0. , 0. ,0)
        
        dt = self.time_step
        self.F = np.array([[1, 0, dt, 0], 
                      [0, 1,  0, dt], 
                      [0, 0,  1, 0],
                      [0, 0,  0, 1]
                      ])
        self.H = np.array([[1, 0, 0 ,0],
                     [0, 1, 0 ,0]]
                     ).reshape(2, 4)
        self.Q = np.array([[0.01, 0.00, 0.00, 0.00], 
                      [0.00, 0.01, 0.00, 0.00], 
                      [0.00, 0.00, 0.01, 0.00],
                      [0.00, 0.00, 0.00, 0.01]
                      ])
        self.R = np.array([
                    [0.1 , 0.1],
                    [0.1 , 0.1]]).reshape(2, 2)

        self.kf = utils.KalmanFilter(F = self.F, H = self.H, Q = self.Q, R = self.R)
        
        self._missing_frame = 0
        self.max_missing_frame = 5

    def predict_kf(self):
        return np.dot(self.H,  self.kf.predict())[0]



class Clustering(Node):

    def __init__(self):
        super().__init__('Clustering')  
        print("ON GOING")
        ### 30 Hz
                
        self.odom_topic_name = '/abwubot/odom'
        self.sub_odom = self.create_subscription(
            Odometry,
            self.odom_topic_name,
            self.get_odometry,
            10)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        
        self.publisher_ = self.create_publisher(MarkerArray, 'Obstacle', 10)
        self.publisher2_= self.create_publisher(MarkerArray, 'Obstacle_ob', 10)

        qos_clock = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.clock_sub  = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile=qos_clock)

        self.ID = 0
        self.Current_group = {}
        self.Previous_group = {}
        self._Start = True
        self.current_object = 0
        self.previous_object = 0

        ########
        self._dth = 0.001
        self._max_cluster_size = 360
        self._euclidean_distance = 0.25

        self.color_track = {}

        self.reset_simulation_service = self.create_service(Empty, 'reset_world', self.service_callback)

        # Create a timer with a period of 1 second (1000 milliseconds)
        ### 30 Hz
        self.timer = self.create_timer(TIME_STEP,  self.timer_callback)

        self.scan = None
        self.time_sec = None
        self.position = None

    def clock_callback(self, msg: Clock):
        # Get current time
        seconds= msg.clock.sec 
        nanoseconds = msg.clock.nanosec 
        self.time_sec = seconds + nanoseconds / 1e9
        # print("TIME" , self.time_sec)

    def service_callback(self, request, response):
        self.get_logger().info('Reseting')
        self.ID = 0
        self.Current_group = {}
        self.Previous_group = {}
        self._Start = True
        self.current_object = 0
        self.previous_object = 0
        self.color_track = {}

        response = Empty.Response()

        return response
    
    def scan_callback(self ,msg):
        self.scan = msg

        

    def timer_callback(self):
        if self.scan == None or self.time_sec == None or self.position == None: return 
        scan = self.scan
        # print(scan)
        x_robot = self.position.x
        y_robot = self.position.y

        orientation_list = [self.orientation.x , self.orientation.y , self.orientation.z , self.orientation.w]
        _ , _ , yaw = euler_from_quaternion(orientation_list)
        robot_pose = x_robot , y_robot , yaw

        group = utils.Clustering(scan)    
        # print(robot_pose)
        # Convert to Global Frame and classify object 
        map_group = []
        _type_group = []
        _center_group = []
        _radius_group = []

        ########## CLASSIFY OBSTABLE AND WALL ############
        # output : ("type" , "center" , "radius")
        # ex if = WALL      return ("WALL", None , None)  
        #         Obstacle  return ("Obstacle" , (x,y)  , radius)  ---> x,y center of circle
        #
        wall_group_combine = []
        wall_group = []

        for g in group : 
            locals_group = []
            for p in g : 
                locals_group.append(utils.transform_local_to_global(robot_pose,p))
                # locals_group.append(p)
                
            
            _type , center , radius = utils.classify_object(locals_group)
            if _type == "Obstacle" : 
                _type_group.append( _type)
                _center_group.append( center)
                _radius_group.append( radius)
            
                map_group.append(locals_group)
            else  :
                ## comebine wall_group to 1 group
                wall_group.append(locals_group)
        
        for i in wall_group : wall_group_combine += i

        #### WALL GROUP ####
        _type_group.append( "WALL")
        _center_group.append( None)
        _radius_group.append( None)
        map_group.append(wall_group_combine)

        self.current_object = len(map_group)

            # if _type == 'Obstacle' : 
            #     print(locals_group)

        # print("+++++++++++++++++++++++++++++")
        # print("Current obstable :" , len([1 for x in _type_group if x == 'Obstacle']))
        # print("Current WALL :" , len([1 for x in _type_group if x == 'WALL']))

        if self._Start :        
            #### assign group by ID
            for i in range(len(map_group)) :

                group = Object()
                
                group.ID = self.ID 
                group.position = map_group[i]
                group.velocity = (0. , 0.) 
                group.center = _center_group[i]
                group.radius = _radius_group[i]
                group.type = _type_group[i] 
                group.time = self.time_sec
                
                if group.type == "Obstacle" :
                    cen_velo = [_center_group[i][0] , _center_group[i][1] , 0. , 0.]
                    group.kf.update(np.array(cen_velo))

                self.Current_group[self.ID] = group


                self.ID +=1


            self._Start = False

            

        else :

            if self.Previous_group != {} : 

                
                
                ######### check associatiob only obstacle
                center_obstable_group = _center_group[:-1]

                ####### Association return (ID , index)
                associations = utils.association(self.Previous_group,center_obstable_group)

                # print(associations)
                for ID , j in associations : 
                    precenter = np.array(self.Previous_group[ID].center)

                    current_center = np.array(_center_group[j])

                    dt = self.time_sec - self.Previous_group[ID].time

                    velo = (current_center - precenter) / dt

                    cen_velo = [current_center[0] , current_center[1] , velo[0] , velo[1]]
                    
                    
                    _x , _y , _vx , _vy = self.Previous_group[ID].predict_kf()

                    self.Current_group[ID].position = map_group[j]
                    self.Current_group[ID].velocity = velo
                    self.Current_group[ID].Predicted_KF = ( _x , _y , _vx , _vy)
                    self.Current_group[ID].center = current_center
                    self.Current_group[ID].radius = _radius_group[j]
                    self.Current_group[ID].kf = copy.deepcopy(self.Previous_group[ID].kf)
                    self.Current_group[ID]._missing_frame = 0

                    # print("ID    " , ID )
                    # print("Distance" , (current_center - precenter) )
                    # print("velo" , velo)
                    # print('velo KF' , self.Current_group[ID].velocity)

                    # if np.linalg.norm(np.array([_x,_y])- current_center) < 1: 
                    #     pass
                    # else : 
                    self.Current_group[ID].type = _type_group[j]
                    self.Current_group[ID].time = self.time_sec
                    self.Current_group[ID].kf.update(np.array(cen_velo))
                
                ### MISSING OBSTACLE 
                #### this maybe obstacle but tracking failed to track so we check again 
                fake_wall = []
                ID_list = [x[0] for x in associations]
                MISSING_ID = []
                for ID in self.Previous_group :
                    if ID not in ID_list : MISSING_ID.append(ID)
                es_center = []
                es_r = []
                for g in wall_group : 
                    try : 
                        (h,k) , radius  = utils.Reconstruct_Circle(g)
                        es_center.append((h,k))
                        es_r.append(radius)
                    except : 
                        es_center.append((np.inf,np.inf))
                        es_r.append(np.inf)
                
                    
                for ID in MISSING_ID:
                    if self.Previous_group[ID].type == "Obstacle" and self.Previous_group[ID] != None and es_center != []   :
                        _x , _y , _vx , _vy = self.Previous_group[ID].predict_kf()
                        r = self.Previous_group[ID].radius
                        _center = np.array((_x,_y))
                        # print(es_center)
                        differences = _center - np.array(es_center)
                        distance = np.sqrt(np.sum(differences**2, axis=1))
                        _argmin_distance = np.argmin(distance)
                        _min_distance = np.min(distance)
                        _index = _argmin_distance
                        # print("MIN" , _min_distance)
                        # print("_index" , _index)

                        if _min_distance  < 0.1 and \
                            abs(r - es_r[_index]) < 0.1 and \
                            abs(es_r[_index]) < 3: 
                            print("THIS IS REAL OBSTACLE ID : {}".format(ID))
                            
                            self.Current_group[ID].position = wall_group[_index]
                            self.Current_group[ID].velocity = (_vx , _vy)
                            self.Current_group[ID].center = (_x,_y)
                            self.Current_group[ID].radius = es_r[_index]
                            self.Current_group[ID].kf = copy.deepcopy(self.Previous_group[ID].kf)
                            self.Current_group[ID]._missing_frame = 0
                            self.Current_group[ID].Predicted_KF = ( _x , _y , _vx , _vy)

                            # if np.linalg.norm(np.array([_x,_y])- current_center) < 1: 
                            #     pass
                            # else : 
                            self.Current_group[ID].type = "Obstacle"
                            self.Current_group[ID].time = self.time_sec

                            precenter = np.array(self.Previous_group[ID].center)

                            current_center = np.array(es_center[_index])

                            dt = self.time_sec -self.Previous_group[ID].time

                            velo = (current_center - precenter) / dt

                            cen_velo = [current_center[0] , current_center[1] , velo[0] , velo[1]]

                            self.Current_group[ID].kf.update(np.array(cen_velo))

                            fake_wall.append(wall_group[_index])

                            self.current_object +=1

                        else : 
                            self.Current_group[ID]._missing_frame +=1
                            if self.Current_group[ID]._missing_frame > self.Current_group[ID].max_missing_frame  :
                                del self.Current_group[ID]

                ####### Maybe New object
                if self.current_object > self.previous_object :
                    index_in_associations = [x[1] for x in associations]
                    ix = [x for x in range(len(map_group)) if x not in index_in_associations]
                    for i in ix :
                        if _center_group[i] != None :
                            group = Object()
                            
                            group.ID = self.ID 
                            group.position = map_group[i]
                            group.velocity = (0. , 0.) 
                            group.center = _center_group[i]
                            group.radius = _radius_group[i]
                            group.type = _type_group[i] 
                            group.time = self.time_sec

                    
                            # if group.type == "Obstacle" :
                            cen_velo = [_center_group[i][0] , _center_group[i][1] , 0. , 0.]
                            group.kf.update(np.array(cen_velo))


                            self.Current_group[self.ID] = group

                            self.ID +=1





                ### Update Wall 
                for ID in self.Current_group :
                    if self.Current_group[ID].type == "WALL" : 
                        wall = []
                        for x in wall_group : 
                            if x not in fake_wall: wall += x
                        self.Current_group[ID].position = wall
                        self.Current_group[ID].velocity = (0. ,0. )
                        self.Current_group[ID].center = None
                        self.Current_group[ID].radius = None

                        # if np.linalg.norm(np.array([_x,_y])- current_center) < 1: 
                        #     pass
                        # else : 
                        self.Current_group[ID].type = "WALL"
                        self.Current_group[ID].time = self.time_sec


                
        self.visualize_grouped_points(self.Current_group)

        if self._Start == False and self.Previous_group != {}:
            print(self.Current_group)
            for ID in self.Current_group:
                if self.Current_group[ID].type == "Obstacle" and ID in self.Previous_group.keys():
                    print("============== ID : {} =================" . format(ID))  
                    print()
                    distance = np.array(self.Current_group[ID].center) - np.array(self.Previous_group[ID].center)
                    dt = self.time_sec - self.Previous_group[ID].time
                    
                    print("Distance :" , distance)
                    print("dt :",dt)
                    print("Center :" , self.Current_group[ID].center)
                    print('velo :' , self.Current_group[ID].velocity)
                    print("Predicted KF :" , self.Current_group[ID].Predicted_KF)

        self.Previous_group = copy.deepcopy(self.Current_group)
        self.previous_object= self.current_object

        # with open('log.npy', 'wb') as f:
        #     np.save(f,self.Current_group)

    def get_odometry(self, odom):
        self.position = odom.pose.pose.position
        self.orientation = odom.pose.pose.orientation
        self.linear_twist = odom.twist.twist.linear
        self.angular_twist = odom.twist.twist.angular


    def visualize_grouped_points(self, point_clusters): ### point_clusters  -> self.Current_group

        marker_array = MarkerArray()
        cg = 0  # Counter for marker IDs
        print("len color Track :" , len(self.color_track))

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
                temp[ID].center = utils.transform_global_to_local(robot_pose,point_clusters[ID].center)

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
        
        self.publisher_.publish(marker_array)

        a = 0
        marker_array2 = MarkerArray()
    
        for ID in temp: 
            if temp[ID].type == "Obstacle" : 
                marker = Marker()

                center = temp[ID].center 
                radius = temp[ID].radius

                marker.header.frame_id = "base_link"
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.scale.x = 1.  # Cylinder diameter
                marker.scale.y = 1.  # Cylinder diameter
                marker.scale.z = 1.  # Cylinder height

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

        self.publisher2_.publish(marker_array2)




def main(args=None):
    rclpy.init(args=args)

    node = Clustering()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
