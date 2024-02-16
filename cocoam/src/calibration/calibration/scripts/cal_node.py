#!/usr/bin/python3


import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np

class CalibrationSensor(Node):
    def __init__(self):
        super().__init__('calibration_sensor')
        # self.create_subscription(Float64MultiArray, "/sensor_data", self.sensor_data_callback, 10)
        self.create_subscription(Imu, "/imu_data", self.imu_data_callback, 10)
        self.collected_data = []

        self.collected_gyro_data = []
        self.collected_acc_data = []
        self.n = 0
        self.num = 1000
        self.isCalibrated = False
    
    def timer_callback(self):
        pass
    
    def imu_data_callback(self, msg):
        # print(msg)
        if self.n < self.num:
            self.n = self.n + 1
            self.collected_acc_data.append([ msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z ])
            self.collected_gyro_data.append( [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z] )
            print(self.n)
        else:
            if self.isCalibrated == False:
                data_gyro_array = np.array( self.collected_gyro_data )
                mean_gyro = np.mean( data_gyro_array, 0 ).tolist()
                cov_gyro = np.cov( data_gyro_array.T )
                
                data_acc_array = np.array( self.collected_acc_data )
                mean_acc = np.mean( data_acc_array, 0 ).tolist()
                cov_acc = np.cov( data_acc_array.T )
                
                print(mean_gyro)
                print("---")
                print(mean_acc)
                print("-----------")
                print(cov_gyro)
                print("---")
                print(cov_acc)
                self.isCalibrated = True


    def sensor_data_callback(self, msg):
        if self.n < self.num:
            self.n = self.n + 1
            self.collected_data.append(msg.data)
            print("collecte data : " + str(self.n))
        else:
            if self.isCalibrated == False:
                data_array = np.array( self.collected_data )
                mean = np.mean( data_array, 0 ).tolist()
                cov = np.cov( data_array.T )
                print(cov)
                self.isCalibrated = True
            

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
