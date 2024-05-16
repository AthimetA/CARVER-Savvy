#!/usr/bin/python3

import os
from signal import signal
import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
import sys, yaml
import numpy as np

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from carversavvy_interfaces.srv import CarversavvyBeginCalibration

class carversavvyCalibrationSensor(Node):
    def __init__(self):
        super().__init__('carversavvyCalibrationSensor')

        # Node Name
        self._name = 'carversavvyCalibrationSensor'

        # Read config file
        self.project_name = 'carversavvy_sensors'
        self.project_path = get_package_share_directory(self.project_name)

        # Config file
        with open(os.path.join(self.project_path, 'config', 'carversavvy_sensor_node_config.yaml'), 'r') as file:
            config = yaml.safe_load(file)

            __SensorGlobal = config['SensorGlobal']

            __carversavvyCalibrationSensor = config[self._name]

            # Sensor Global Parameters
            self.imu_topic = __SensorGlobal['IMU_TOPIC_NAME']
            self.imu_path = os.path.join(self.project_path, __SensorGlobal['IMU_CALIBRATION_FILE'])
            self.wheel_path = os.path.join(self.project_path, __SensorGlobal['WHEEL_VELOCITY_CALIBRATION_FILE'])

            # carversavvyRCalibrationSensor Parameters
            self.max_n = __carversavvyCalibrationSensor['MAX_CALIBRATION_ITERATIONS']
            self.node_calibration_srv_name = __carversavvyCalibrationSensor['NODE_CALIBRATION_SERVICE_NAME']
        
        
        # IMU subscriber
        self.sensor_subscriber = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.imu_linear_acceleration_list = np.array([])
        self.imu_angular_velocity_list = np.array([])
        self.imu_n = 0
        self.imu_calibration = False

        # Wheel velocity subscriber
        self.create_subscription(Float64, '/wheelL_vel', self.wheelL_callback, 10)
        self.create_subscription(Float64, '/wheelR_vel', self.wheelR_callback, 10)
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.left_wheel_velocity_list = np.array([])
        self.right_wheel_velocity_list = np.array([])
        self.wheel_n = 0
        self.wheel_calibration = False
        
        # Service
        self.srv = self.create_service(CarversavvyBeginCalibration, self.node_calibration_srv_name , self.begin_calibration_callback)

        self.get_logger().info('carversavvyCalibrationSensor node has been started')
        srv_text ='ros2 service call /carversavvy_begin_calibration carversavvy_interfaces/srv/carversavvyBeginCalibration "{imu_calibration: false, wheel_calibration: false}"'
        self.get_logger().info(f'Waiting for service call: '+srv_text)
        
    def imu_callback(self,msg: Imu):
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        if self.imu_calibration == True and self.imu_n < self.max_n:
            self.imu_angular_velocity_list = np.append(self.imu_angular_velocity_list, angular_velocity)
            self.imu_linear_acceleration_list = np.append(self.imu_linear_acceleration_list, linear_acceleration)
            self.imu_n += 1
            self.get_logger().info(f'IMU calibration: {self.imu_n}/{self.max_n}')
        elif self.imu_calibration == True and self.imu_n == self.max_n:
            self.imu_angular_velocity_list = self.imu_angular_velocity_list.reshape(self.max_n,3)
            self.imu_linear_acceleration_list = self.imu_linear_acceleration_list.reshape(self.max_n,3)
            self.imu_linear_acceleration_mean = np.mean(self.imu_linear_acceleration_list, axis=0)
            self.imu_angular_velocity_mean = np.mean(self.imu_angular_velocity_list, axis=0)
            self.imu_linear_acceleration_cov = np.cov(self.imu_linear_acceleration_list.T)
            self.imu_angular_velocity_cov = np.cov(self.imu_angular_velocity_list.T)
            self.imu_calibration = False
            self.get_logger().info('IMU calibration finished')
            self.get_logger().info(f'IMU linear acceleration mean: {self.imu_linear_acceleration_mean}')
            self.get_logger().info(f'IMU angular velocity mean: {self.imu_angular_velocity_mean}')
            self.get_logger().info(f'IMU linear acceleration covariance: {self.imu_linear_acceleration_cov}')
            self.get_logger().info(f'IMU angular velocity covariance: {self.imu_angular_velocity_cov}')
            # Write to yaml
            self.get_logger().info(f'Writing to {self.imu_path}')
            with open(self.imu_path, 'w') as f:
                data = {'imu_linear_acceleration_mean': self.imu_linear_acceleration_mean.tolist(), 'imu_angular_velocity_mean': self.imu_angular_velocity_mean.tolist(), 'imu_linear_acceleration_cov': self.imu_linear_acceleration_cov.tolist(), 'imu_angular_velocity_cov': self.imu_angular_velocity_cov.tolist()}
                yaml.dump(data, f)
        else:
            pass
        
    def wheelL_callback(self, msg: Float64):
        self.left_wheel_velocity = msg.data
    
    def wheelR_callback(self, msg: Float64):
        self.right_wheel_velocity = msg.data

        left_wheel_velocity = self.left_wheel_velocity
        right_wheel_velocity = self.right_wheel_velocity
        if self.wheel_calibration == True and self.wheel_n < self.max_n:
            self.left_wheel_velocity_list = np.append(self.left_wheel_velocity_list, left_wheel_velocity)
            self.right_wheel_velocity_list = np.append(self.right_wheel_velocity_list, right_wheel_velocity)
            self.wheel_n += 1
            self.get_logger().info(f'Wheel calibration: {self.wheel_n}/{self.max_n}')
        elif self.wheel_calibration == True and self.wheel_n == self.max_n:
            self.left_wheel_velocity_list = self.left_wheel_velocity_list.reshape(self.max_n,1)
            self.right_wheel_velocity_list = self.right_wheel_velocity_list.reshape(self.max_n,1)
            self.left_wheel_velocity_mean = np.mean(self.left_wheel_velocity_list, axis=0)
            self.right_wheel_velocity_mean = np.mean(self.right_wheel_velocity_list, axis=0)
            self.left_wheel_velocity_cov = np.cov(self.left_wheel_velocity_list.T)
            self.right_wheel_velocity_cov = np.cov(self.right_wheel_velocity_list.T)
            self.wheel_calibration = False
            # Write to yaml
            self.get_logger().info('Wheel calibration finished')
            self.get_logger().info(f'Left wheel velocity mean: {self.left_wheel_velocity_mean}')
            self.get_logger().info(f'Right wheel velocity mean: {self.right_wheel_velocity_mean}')
            self.get_logger().info(f'Left wheel velocity covariance: {self.left_wheel_velocity_cov}')
            self.get_logger().info(f'Right wheel velocity covariance: {self.right_wheel_velocity_cov}')
            with open(self.wheel_path, 'w') as f:
                data = {'left_wheel_velocity_mean': self.left_wheel_velocity_mean.tolist(), 'right_wheel_velocity_mean': self.right_wheel_velocity_mean.tolist(), 'left_wheel_velocity_cov': self.left_wheel_velocity_cov.tolist(), 'right_wheel_velocity_cov': self.right_wheel_velocity_cov.tolist()}
                yaml.dump(data, f)
        else:
            pass

    def begin_calibration_callback(self, request: CarversavvyBeginCalibration.Request, response: CarversavvyBeginCalibration.Response):

        self.get_logger().info('Calibration service called')
        self.imu_calibration = request.imu_calibration
        self.wheel_calibration = request.wheel_calibration
        call_back_text = "Sensor calibration: "

        if self.imu_calibration == True:
            self.imu_angular_velocity_list = np.array([])
            self.imu_linear_acceleration_list = np.array([])
            self.imu_n = 0
            call_back_text += "IMU calibration"
        if self.wheel_calibration == True:
            self.left_wheel_velocity_list = np.array([])
            self.right_wheel_velocity_list = np.array([])
            self.wheel_n = 0
            call_back_text += "Wheel calibration"

        response.node_name = self.get_name()
        response.call_back_status = call_back_text
        self.get_logger().info(call_back_text)
        return response

    
def main(args=None):
    rclpy.init(args=args)
    node = carversavvyCalibrationSensor()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
