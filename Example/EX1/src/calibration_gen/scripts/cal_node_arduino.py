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

class CalibrationSensor(Node):
    def __init__(self):
        super().__init__('calibration_sensor')
        
        # establish timer
        self.timer_period = 0.1
        self.sensor_subscriber = self.create_subscription(Imu,'/imu_data',self.sensor_callback,10)

        # Calibration process
        self.angular_velocity_list = []
        self.linear_acceleration_list = []
        self.n = 0
        self.max_n = (1/0.02) * 10
        self.isCalibrated = False

        # YAML save path
        self.path = '/home/athimet/FRA532_Mobile_Robot/EX1/src/calibration_gen/config/imu_calibration.yaml'
        
    def sensor_callback(self,msg):
        # process the data
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        if self.isCalibrated == False and self.n < self.max_n:
            self.angular_velocity_list.append(angular_velocity)
            self.linear_acceleration_list.append(linear_acceleration)
            self.n += 1
            print(f'Cailbrationg iteration: {self.n}/{self.max_n}')
            print(f'Angular Velocity: {angular_velocity}')
            print(f'Linear Acceleration: {linear_acceleration}')
            print('-----------------------------------')
        elif self.isCalibrated == False and self.n == self.max_n:
            self.angular_velocity_list = np.array(self.angular_velocity_list)
            self.linear_acceleration_list = np.array(self.linear_acceleration_list)
            self.linear_acceleration_mean = np.mean(self.linear_acceleration_list, axis=0)
            self.angular_velocity_mean = np.mean(self.angular_velocity_list, axis=0)
            self.linear_acceleration_cov = np.cov(self.linear_acceleration_list.T)
            self.angular_velocity_cov = np.cov(self.angular_velocity_list.T)
            self.isCalibrated = True
            print('Calibration is done')
            print(f'Linear Acceleration Mean: \n{self.linear_acceleration_mean}')
            print(f'Angular Velocity Mean: \n{self.angular_velocity_mean}')
            print(f'Linear Acceleration Covariance: \n{self.linear_acceleration_cov}')
            print(f'Angular Velocity Covariance: \n{self.angular_velocity_cov}')
            # Write to yaml
            with open(self.path, 'w') as f:
                data = {'linear_acceleration_mean': self.linear_acceleration_mean.tolist(), 'angular_velocity_mean': self.angular_velocity_mean.tolist(), 'linear_acceleration_cov': self.linear_acceleration_cov.tolist(), 'angular_velocity_cov': self.angular_velocity_cov.tolist()}
                yaml.dump(data, f)

        else:
            pass

    def timer_callback(self):
        pass
    
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationSensor()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
