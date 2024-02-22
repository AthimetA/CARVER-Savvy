#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports
from sensor_msgs.msg import Imu
import numpy as np
import os
import yaml

class IMUSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        # if config file exists, run in calibration mode
        self.config_path = '/home/athimet/FRA532_Mobile_Robot/EX1/src/calibration_gen/config/imu_calibration.yaml'
        if os.path.exists(self.config_path):
            with open(self.config_path, 'r') as f:
                config = yaml.load(f, Loader=yaml.FullLoader)
                # Offset values
                self.gx_offset = config["angular_velocity_mean"][0]
                self.gy_offset = config["angular_velocity_mean"][1]
                self.gz_offset = config["angular_velocity_mean"][2]
                self.linear_acceleration_x_offset = config["linear_acceleration_mean"][0]
                self.linear_acceleration_y_offset = config["linear_acceleration_mean"][1]
                self.linear_acceleration_z_offset = config["linear_acceleration_mean"][2]
                # Covariance matrices
                self.linear_acceleration_cov = np.array(config["linear_acceleration_cov"])
                self.angular_velocity_cov = np.array(config["angular_velocity_cov"])
                self.get_logger().info('Calibration data found. Running in calibration mode.')
                # Print calibration data
                self.get_logger().info(f'Linar Acceleration offset: \n{self.linear_acceleration_x_offset}, {self.linear_acceleration_y_offset}, {self.linear_acceleration_z_offset}')
                self.get_logger().info(f'Linear Acceleration Covariance: \n{self.linear_acceleration_cov}')
                self.get_logger().info(f'Angular Velocity offset: \n{self.gx_offset}, {self.gy_offset}, {self.gz_offset}')
                self.get_logger().info(f'Angular Velocity Covariance: \n{self.angular_velocity_cov}')
        else:
            # self.gx_offset = 2.1259300000000003
            # self.gy_offset = -0.46462
            # self.gz_offset = -1.83018
            self.gx_offset = 0
            self.gy_offset = 0
            self.gz_offset = 0
            self.linear_acceleration_x_offset = 0
            self.linear_acceleration_y_offset = 0
            self.linear_acceleration_z_offset = 0
            self.get_logger().info('No calibration data found. Running in normal mode.')

        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.read_and_publish)
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        self.publisher_imu = self.create_publisher(Imu, '/imu_data', 10)

        # [INFO] [1707379980.026784490] [imu_calibration_node]: Calibration complete: 
        # GX offset: 2.0350999999999995, 
        # GY offset: 0.051550000000000006, 
        # GZ offset: -1.6954200000000001

    def read_and_publish(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            parts = line.split('\t')
            if len(parts) == 6:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"  # Adjust as needed
                # Gyroscope data in rad/s
                imu_msg.angular_velocity.x = float(parts[0]) - self.gx_offset
                imu_msg.angular_velocity.y = float(parts[1]) - self.gy_offset
                imu_msg.angular_velocity.z = float(parts[2]) - self.gz_offset
                # imu_msg.angular_velocity_covariance = self.angular_velocity_cov
                # imu_msg.angular_velocity_covariance = np.array( [   0.001, 0.001, 0.001, 
                #                                                     0.001, 0.001, 0.001, 
                #                                                     0.001, 0.001, 0.001,])
                # Accelerometer data in m/s^2
                imu_msg.linear_acceleration.x = float(parts[3]) - self.linear_acceleration_x_offset
                imu_msg.linear_acceleration.y = float(parts[4]) - self.linear_acceleration_y_offset
                imu_msg.linear_acceleration.z = float(parts[5]) - self.linear_acceleration_z_offset
                # imu_msg.linear_acceleration_covariance = self.linear_acceleration_cov
                # imu_msg.linear_acceleration_covariance = np.array( [   0.001, 0.001, 0.001, 
                #                                                     0.001, 0.001, 0.001, 
                #                                                     0.001, 0.001, 0.001,])

                self.publisher_imu.publish(imu_msg)
                self.get_logger().info(f'Publishing IMU Data: {line}')
                # self.get_logger().info(f'Publishing IMU Data: {imu_msg}')
            else:
                self.get_logger().error('Unexpected data format received.')


def main(args=None):
    rclpy.init(args=args)
    imu_serial_reader = IMUSerialReader()
    rclpy.spin(imu_serial_reader)
    imu_serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()