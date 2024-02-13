#!/usr/bin/python3

import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np
from sensor_msgs.msg import Imu
class CalibrationGen(Node):
    def __init__(self):
        super().__init__('signal_receive_calibrate')
        
        # establish timer
        self.timer_period = 0.02
        self.sensor_subscriber = self.create_subscription(Imu,'/imu_data',self.imu_data_callback,10)
        self.timer = self.create_timer(self.timer_period,self.timer_callback)

        # init data
        self.imu_accel_data = []
        self.imu_gyro_data = []
        self.n0 = 0
        self.n_max = 500
        self.calibrate_flag = False

    def timer_callback(self):
        pass

    def imu_data_callback(self,msg):
        # update sensor data
        if self.n0 < self.n_max:
            self.n0 += 1
            self.imu_accel_data.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            self.imu_gyro_data.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            print("Received Data No." + str(self.n0))
            print([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            print([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        elif self.calibrate_flag == False:
            imu_accel_array = np.array(self.imu_accel_data)
            imu_accel_mean_data = np.mean(imu_accel_array,0).tolist()
            imu_accel_cov_matrix = np.cov(imu_accel_array.T).tolist()
            print("IMU Accel Mean: \n", imu_accel_mean_data)
            print("IMU Accel Covariance: \n", imu_accel_cov_matrix)

            imu_gyro_array = np.array(self.imu_gyro_data)
            imu_gyro_mean_data = np.mean(imu_gyro_array,0).tolist()
            imu_gyro_cov_matrix = np.cov(imu_gyro_array.T).tolist()
            print("IMU Gyro Mean: \n", imu_gyro_mean_data)
            print("IMU Gyro Covariance: \n", imu_gyro_cov_matrix)
            
            #write to yaml
            path = '/home/duplicix/FRA532_Mobile_Robot/EX1_B/src/calibration_gen/config/imu_calibration.yaml'
            with open(path, 'w') as f:
                yaml.dump({'accel_mean': imu_accel_mean_data, 'accel_cov': imu_accel_cov_matrix,
                           'gyro_mean': imu_gyro_mean_data, 'gyro_cov': imu_gyro_cov_matrix}, f)
            
            print("Calibration complete")
            self.calibrate_flag = True
    
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationGen()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()