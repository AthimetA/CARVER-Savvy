#!/usr/bin/python3


import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np

class CalibrationSensor(Node):
    def __init__(self):
        super().__init__('calibration_sensor')
        
        # establish timer
        self.timer_period = 0.1
        self.sensor_subscriber = self.create_subscription(Float64MultiArray,'/sensor_data',self.sensor_callback,10)

        # Calibration process
        self.data_list = []
        self.n = 0
        self.max_n = 10000
        self.isCalibrated = False

        # YAML save path
        calibration_gen_path = get_package_share_directory('zhbbot_sensors')
        self.path = os.path.join(calibration_gen_path, 'config', 'calibration.yaml')
        
    def sensor_callback(self,msg):
        # process the data
        msg = np.array(msg.data)
        if self.isCalibrated == False and self.n < self.max_n:
            self.data_list.append(msg)
            self.n += 1
            print(f'Cailbrationg iteration: {self.n}/{self.max_n}')
        elif self.isCalibrated == False and self.n == self.max_n:
            self.data_list = np.array(self.data_list)
            self.mean = np.mean(self.data_list, axis=0)
            self.cov = np.cov(self.data_list, rowvar=False)
            self.isCalibrated = True
            with open(self.path, 'w') as f:
                yaml.dump({'mean': self.mean.tolist(), 'covariance': self.cov.tolist()}, f)
                print(f'Saved the calibration data to {self.path}')
            print('Calibration is done')
            print('Mean:\n', self.mean)
            print('Covariance:\n', self.cov)
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
