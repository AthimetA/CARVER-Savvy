#!/usr/bin/python3

import os
from signal import signal
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np

class PMZBRosTest(Node):
    def __init__(self):
        super().__init__('PMZBRosTest')

        # Read config file
        self.project_name = 'pmzbbot_sensors'
        self.project_path = get_package_share_directory(self.project_name)

        with open(os.path.join(self.project_path, 'config', 'pmzb_sensor_node_config.yaml'), 'r') as file:
            config = yaml.safe_load(file)
            key = config.keys()
            print(f'Keys: {key}')
            PMZBRosBridge = config['PMZBRosBridge']
            PMZBCalibrationSensor = config['PMZBCalibrationSensor']
            print(f'PMZBRosBridge: {PMZBRosBridge}')
            print(f'PMZBCalibrationSensor: {PMZBCalibrationSensor}')
    
def main(args=None):
    rclpy.init(args=args)
    node = PMZBRosTest()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
