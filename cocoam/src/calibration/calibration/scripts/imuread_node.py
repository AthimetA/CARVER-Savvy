#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports
from sensor_msgs.msg import Imu
import numpy as np

class IMUSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.read_and_publish)
        self.ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
        self.publisher_imu = self.create_publisher(Imu, 'imu_data', 10)
        self.publisher_imu_cal = self.create_publisher(Imu, 'imu_data_cal', 10)
        self.gx_offset = 0.04000000000000003
        self.gy_offset = -0.08000000000000006
        self.gz_offset = -0.07000000000000009

        self.ax_offset = 0.21720000000000017
        self.ay_offset = -0.39280000000000004
        self.az_offset = 9.880899999999997


        # [INFO] [1707379980.026784490] [imu_calibration_node]: Calibration complete: 
        # GX offset: 2.0350999999999995, 
        # GY offset: 0.051550000000000006, 
        # GZ offset: -1.6954200000000001

    def read_and_publish(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                parts = line.split('\t')
                if len(parts) == 6:
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = "imu_link"  # Adjust as needed
                    # Gyroscope data in rad/s
                    imu_msg.angular_velocity.x = float(parts[0])
                    imu_msg.angular_velocity.y = float(parts[1])
                    imu_msg.angular_velocity.z = float(parts[2])
                    # imu_msg.angular_velocity_covariance = np.array( [   9.88888889e-06, -2.22222222e-07, 9.09090909e-07, 
                    #                                                     -2.22222222e-07, 1.97979798e-06, -2.02020202e-07,
                    #                                                     9.09090909e-07, -2.02020202e-07, 9.09090909e-06])
                    # Accelerometer data in m/s^2
                    imu_msg.linear_acceleration.x = float(parts[3])
                    imu_msg.linear_acceleration.y = float(parts[4])
                    imu_msg.linear_acceleration.z = float(parts[5])
                    # imu_msg.linear_acceleration_covariance = np.array( [   0.001, 0.001, 0.001, 
                    #                                                     0.001, 0.001, 0.001, 
                    #                                                     0.001, 0.001, 0.001,])

                    imu_cal_msg = Imu()
                    imu_cal_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_cal_msg.header.frame_id = "imu_link"  # Adjust as needed
                    # Gyroscope data in rad/s
                    imu_cal_msg.angular_velocity.x = float(parts[0]) - self.gx_offset
                    imu_cal_msg.angular_velocity.y = float(parts[1]) - self.gy_offset
                    imu_cal_msg.angular_velocity.z = float(parts[2]) - self.gz_offset
                    imu_cal_msg.angular_velocity_covariance = np.array( [   9.88888889e-06, -2.22222222e-07, 9.09090909e-07, 
                                                                        -2.22222222e-07, 1.97979798e-06, -2.02020202e-07,
                                                                        9.09090909e-07, -2.02020202e-07, 9.09090909e-06])
                    # Accelerometer data in m/s^2
                    imu_cal_msg.linear_acceleration.x = float(parts[3]) - self.ax_offset
                    imu_cal_msg.linear_acceleration.y = float(parts[4]) - self.ay_offset
                    imu_cal_msg.linear_acceleration.z = float(parts[5]) - self.az_offset
                    # imu_msg.linear_acceleration_covariance = np.array( [   0.001, 0.001, 0.001, 
                    #                                                     0.001, 0.001, 0.001, 
                    #                                                     0.001, 0.001, 0.001,])

                    self.publisher_imu.publish(imu_msg)
                    self.publisher_imu_cal.publish(imu_cal_msg)
                    self.get_logger().info(f'Publishing IMU Data: {line}')
                else:
                    self.get_logger().error('Unexpected data format received.')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    imu_serial_reader = IMUSerialReader()
    rclpy.spin(imu_serial_reader)
    imu_serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()