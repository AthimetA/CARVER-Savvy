#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import random
from scipy.optimize import linear_sum_assignment

class Clustering(Node):

    def __init__(self):
        super().__init__('Clustering')
        self.position = Pose()       

        self.odom_topic_name = 'abwubot/odom'
        self.sub_odom = self.create_subscription(
            Odometry,
            self.odom_topic_name,
            self.get_odometry,
            10)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        self.ID = 0
        self.Current_group = {}
        self.Previous_group = {}
        self._Start = True

        ########
        self._dth = 0.2
        self._max_cluster_size = 360
        self._euclidean_distance = 0.25

        self.color_track = {}

    def listener_callback(self, msg):
        scan = msg

        if self._Start :        
            group = self.Clustering(scan)
            #### assign group by ID
            for i in group :
                self.Current_group[self.ID] = {"position" : i}
                self.ID +=1

            
            self.Previous_group = self.Current_group

            self.visualize_grouped_points(self.Current_group)

            self._Start = False
        else :
            group = self.Clustering(scan)
        
            if self.Previous_group != {} :
                associations = self.association(self.Previous_group,group)

                ### update 
                #### missing or new
                if len(associations) != len(group) : ## NEW OBJECT
                    if len(group) > len(associations) :
                        index_in_associations = [x[1] for x in associations]
                        ix = [x for x in range(len(group)) if x not in index_in_associations]
                        for i in ix :
                            self.Current_group[self.ID] = {"position" : group[i]}
                            self.ID +=1 
                    else : ### MISSING
                        index_in_associations = [x[0] for x in associations]
                        ix = [x for x in range(len(self.Previous_group)) if x not in index_in_associations]
                        ID_list= [] 
                        for i in ix :  
                            ID_list.append(list(self.self.Previous_group)[i])
                        for i in ID_list :
                            del self.Current_group[i]  

                ### normal 
                else : 
                    for i , j in associations : 
                        self.Current_group[i] = {"position" : group[j]}


                self.visualize_grouped_points(self.Current_group)

            self.Previous_group = self.Current_group
        print("current object :", len(self.Current_group))

        # with open('test.npy', 'wb') as f:
        #     np.save(f, np.array(pos))
        #     np.save(f, np.array(distance))
        #     np.save(f, np.array(angel))

        # with open('scan.npy', 'wb') as f:
        #     np.save(f, np.array(scan))


    def Clustering(self,scan_in):
        clusters = []
        scan = scan_in
        dth = self._dth
        c_points = 0
        
        # Find the number of non inf laser scan values and save them in c_points
        for i in range(len(scan.ranges)):
            if not math.isinf(scan.ranges[i]):
                c_points += 1
        
        polar = [[0.0, 0.0] for _ in range(c_points + 1)]  # c_points+1 for wrapping
        j = 0
        for i in range(len(scan.ranges)):
            if not math.isinf(scan.ranges[i]):
                polar[j][0] = scan.ranges[i]  # first column is the range 
                polar[j][1] = scan.angle_min + i * scan.angle_increment  # second angle in rad
                j += 1
        
        # Complete the circle
        polar[-1][0] = polar[0][0]
        polar[-1][1] = polar[0][1]

        clustered1 = [False] * (c_points + 1)  # change to true when it is the first of the cluster
        clustered2 = [False] * (c_points + 1)  # change to true when it is clustered by another one

        for i in range(c_points):
            d = math.sqrt(polar[i][0]**2 + polar[i+1][0]**2 - 2 * polar[i][0] * polar[i+1][0] * math.cos(polar[i+1][1] - polar[i][1]))
            if d < dth:
                clustered1[i] = True  # both points belong to clusters
                clustered2[i+1] = True

        # If the last(first also) point is clustered and the first is not, make the first clustered
        if clustered2[-1] and not clustered2[0]:
            clustered2[0] = True
            clustered1[0] = False

        begin = []
        nclus = []
        i = 0
        flag = True

        while i < c_points and flag:
            if clustered1[i] and not clustered2[i] and flag:
                begin.append(i)
                nclus.append(1)
                while i+1 < c_points and clustered2[i+1] and clustered1[i+1]:
                    i += 1
                    nclus[-1] += 1
                    if i == c_points-1 and flag:
                        i = -1
                        flag = False
                nclus[-1] += 1
            i += 1

        polar.pop()  # remove the wrapping element
        len_polar = len(polar)

        for i in range(len(begin)):
            cluster = []
            j = begin[i]
            flag = True
            while j < nclus[i] + begin[i]:
                if j == len_polar and flag:
                    flag = False
                if flag:
                    x = polar[j][0] * math.cos(polar[j][1])
                    y = polar[j][0] * math.sin(polar[j][1])
                else:
                    x = polar[j-len_polar][0] * math.cos(polar[j-len_polar][1])
                    y = polar[j-len_polar][0] * math.sin(polar[j-len_polar][1])
                cluster.append((x, y))
                j += 1
            clusters.append(cluster)

        return clusters


    def get_odometry(self, odom: Odometry):
        self.position = odom.pose.pose.position
        self.orientation = odom.pose.pose.orientation
        self.linear_twist = odom.twist.twist.linear
        self.angular_twist = odom.twist.twist.angular


    def visualize_grouped_points(self, point_clusters): ### point_clusters  -> self.Current_group
        marker_array = MarkerArray()
        cg = 0  # Counter for marker IDs

        for i in point_clusters : 
            if i not in self.color_track : 
                self.color_track[i] = [random.random() , random.random() ,random.random()]

        for ID in point_clusters:
            gpoints = Marker()
            gpoints.header.frame_id = 'base_link'  # Replace 'base_link' with your desired frame
            gpoints.header.stamp = self.get_clock().now().to_msg()
            gpoints.ns = 'clustered_points'
            gpoints.action = Marker.ADD
            gpoints.pose.orientation.w = 1.0
            gpoints.type = Marker.POINTS
            gpoints.scale.x = 0.04
            gpoints.scale.y = 0.04

            color = self.color_track[ID]
            
            gpoints.color.r = color[0]
            gpoints.color.g = color[1]
            gpoints.color.b = color[2]
            gpoints.color.a = 1.0

            for point in point_clusters[ID]["position"]:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0.0
                gpoints.points.append(p)

            gpoints.id = cg
            cg += 1
            marker_array.markers.append(gpoints)
        
        self.publisher_.publish(marker_array)


    def association(self,previous_group,group):
        
        previous_mean = []
        current_mean  = []
        for g in previous_group :
            x = sum([i[0] for i in previous_group[g]['position']])
            y = sum([i[1] for i in previous_group[g]['position']])
            previous_mean.append([x,y])

        for g in group:
            x = sum([i[0] for i in g])
            y = sum([i[1] for i in g])
            current_mean.append([x,y])
        euclidean = np.zeros((len(previous_group),len(group)))
        

        for i in range(len(previous_group)):
            for j in range(len(group)):
                euclidean[i, j] = np.linalg.norm(np.array(previous_mean[i]) - np.array(current_mean[j]))

        self.get_logger().info(f'euclidean distance: {euclidean}')
                
        
        # Solve the assignment problem using the Hungarian algorithm
        row_indices, col_indices = linear_sum_assignment(euclidean)
        
        # Generate associations between objects based on the indices
        associations = [(row_indices[i], col_indices[i]) for i in range(len(row_indices))]
        return associations


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Clustering()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
