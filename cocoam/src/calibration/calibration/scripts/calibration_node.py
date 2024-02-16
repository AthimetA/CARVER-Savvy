#!/usr/bin/python3

from concurrent.futures import Executor
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from calibration_interfaces.action import Calibrate
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
import os


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__('calibration_server')
        self.rate = self.create_rate(10)
        self.action_server = ActionServer(self,Calibrate,'/calibrate',self.execute_callback)
        
        self.current_data = Float64MultiArray()     # current data (single time-stamp)
        self.collected_data = []                    # accumulated data
        
    def execute_callback(self,goal_handle):
        print("test")
        self.get_logger().info(f'Executing action...')
        self.collected_data = []
        feedback_msg = Calibrate.Feedback()
        num = goal_handle.request.num

        for i in range(num):
            data = self.current_data
            self.collected_data.append(data)
            feedback_msg.data = data
            goal_handle.publish_feedback(feedback_msg)
            self.rate.sleep()
        goal_handle.succeed()

        data_array = np.array( self.collected_data )
        
        # return absolute distance as result
        result = Calibrate.Result()
        result.mean = np.mean(data_array,0).tolist()
        cov = np.cov(data_array.T)
        result.covariance = np.reshape(cov, (cov.shape[0] * cov.shape[1])).tolist()
        calibration_path = get_package_share_directory('calibration')
        file = os.path.join(calibration_path, 'config', 'sensor_properties.yaml')
        with open(file,'w') as f:
            yaml.dump( {'mean': result.mean, 'covariance': result.covariance},f )
        os.system("gedit " + file)
        return result


class Calibration(Node):
    def __init__(self,action_server:CalibrationActionServer):
        super().__init__('calibration_subscriber')
        self.sensor_subscription = self.create_subscription(Float64MultiArray, '/sensor_data', self.sensor_callback, 10)
        self.action_server = action_server
        
    def sensor_callback(self,msg):
        self.action_server.current_data = msg.data

def main(args=None):
    rclpy.init(args=args)
    try:
        action_server = CalibrationActionServer()
        node = Calibration(action_server=action_server)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.add_node(action_server)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            action_server.destroy_node()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()
