#!/usr/bin/python3

import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np
class CalibrationGen(Node):
    def __init__(self):
        super().__init__('signal_receive_calibrate')
        
        # establish timer
        self.timer_period = 0.01
        self.sensor_subscriber = self.create_subscription(Float64MultiArray,'/sensor_data',self.sensor_data_callback,10)
        self.timer = self.create_timer(self.timer_period,self.timer_callback)

        # init data
        self.sensor_data = []
        self.n0 = 0
        self.n_max = 10000
        self.calibrate_flag = False

    def timer_callback(self):
        pass

    def sensor_data_callback(self,msg):
        # update sensor data
        if self.n0 < self.n_max:
            self.n0 += 1
            self.sensor_data.append(msg.data)
            print("Received Data No." + str(self.n0))
            print(msg.data)
        elif self.calibrate_flag == False:
            data_array = np.array(self.sensor_data)
            mean_data = np.mean(data_array,0).tolist()
            cov_matrix = np.cov(data_array.T)
            print("Mean: ", mean_data)
            print("Covariance: ", cov_matrix)
            self.calibrate_flag = True
            
        
    
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationGen()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()