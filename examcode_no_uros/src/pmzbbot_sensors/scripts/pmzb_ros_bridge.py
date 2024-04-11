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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformStamped

from pmzbbot_interfaces.srv import PmzbbotBeginCalibration

NS_TO_SEC= 1000000000

class PMZBRosBridge(Node):
    def __init__(self):
        super().__init__('PMZBRosBridge')

        # Node Name
        self._name = 'PMZBRosBridge'

        # Read config file
        self.project_name = 'pmzbbot_sensors'
        self.project_path = get_package_share_directory(self.project_name)

        with open(os.path.join(self.project_path, 'config', 'pmzb_sensor_node_config.yaml'), 'r') as file:
            config = yaml.safe_load(file)

            __SensorGlobal = config['SensorGlobal']

            __PMZBRosBridge = config[self._name]

            # Sensor Global Parameters
            self.imu_sub_topic = __SensorGlobal['IMU_TOPIC_NAME']
            self.wheel_vel_sub_topic = __SensorGlobal['WHEEL_VELOCITY_TOPIC_NAME']
            self.timer_period = 1.0 / __SensorGlobal['TIMER_FREQUENCY']
            self.imu_calibration_file_name = __SensorGlobal['IMU_CALIBRATION_FILE']


            # PMZBRosBridge Parameters
            self.imu_pub_topic = __PMZBRosBridge['IMU_PUB_TOPIC_NAME']
            self.wheel_odom_pub_topic = __PMZBRosBridge['WHEEL_ODOMETRY_PUB_TOPIC_NAME']
            self._WHEEL_RADIUS = __PMZBRosBridge['WHEEL_RADIUS']
            self._BASE_WIDTH = __PMZBRosBridge['BASE_WIDTH']
            self.odom_frame_id = __PMZBRosBridge['ODOM_FRAME_ID']
            self.base_frame_id = __PMZBRosBridge['BASE_FRAME_ID']
            self.imu_frame_id = __PMZBRosBridge['IMU_FRAME_ID']

        # IMU subscriber
        self.imu_sub = self.create_subscription(Twist, self.imu_sub_topic, self.imu_callback, 10)
        # Load the calibration parameters
        imu_config_path = os.path.join(self.project_path, self.imu_calibration_file_name)
        if os.path.exists(imu_config_path):
            with open(imu_config_path, 'r') as file:
                imu_calibration = yaml.safe_load(file)
                # Load the calibration parameters
                # Offset values
                self.gx_offset = imu_calibration["imu_angular_velocity_mean"][0]
                self.gy_offset = imu_calibration["imu_angular_velocity_mean"][1]
                self.gz_offset = imu_calibration["imu_angular_velocity_mean"][2]
                self.linear_acceleration_x_offset = imu_calibration["imu_linear_acceleration_mean"][0]
                self.linear_acceleration_y_offset = imu_calibration["imu_linear_acceleration_mean"][1]
                self.linear_acceleration_z_offset = imu_calibration["imu_linear_acceleration_mean"][2]
                # Covariance matrices
                self.linear_acceleration_cov = np.array(imu_calibration["imu_linear_acceleration_cov"]).reshape(9)
                self.angular_velocity_cov = np.array(imu_calibration["imu_angular_velocity_cov"]).reshape(9)
                self.get_logger().info('PMZBRosBridge initialized with IMU calibration data.')
        else:
            self.get_logger().info('PMZBRosBridge initialized without IMU calibration data.')
            self.gx_offset = 0.0
            self.gy_offset = 0.0
            self.gz_offset = 0.0
            self.linear_acceleration_x_offset = 0.0
            self.linear_acceleration_y_offset = 0.0
            self.linear_acceleration_z_offset = 0.0
            self.linear_acceleration_cov = np.zeros(9)
            self.angular_velocity_cov = np.zeros(9)

        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, self.imu_pub_topic, 10)

        # Wheel velocity subscriber
        self.wheel_vel_sub = self.create_subscription(Twist, self.wheel_vel_sub_topic, self.wheel_vel_callback, 10)
        self.x = 0.0
        self.y = 0.0
        self.wz = 0.0
        self.pose = np.matrix([0.0, 0.0, 0.0]).T # x, y, theta
        self.pose_cov = np.diag([0.1, 0.1, 0.1])

        # Wheel odometry publisher
        self.wheel_odom_pub = self.create_publisher(Odometry, self.wheel_odom_pub_topic, 10)
        self.odom_broadcaster = TransformBroadcaster(self)

        # Timer to publish IMU data and wheel odometry
        self.create_timer(self.timer_period, self.timer_callback)
        self.imu_msg = Imu()
        self.odom_msg = Odometry()
        self.time_last = self.get_clock().now()

    def imu_callback(self, msg: Twist):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id
        # Gyroscope data in rad/s
        imu_msg.angular_velocity.x = msg.angular.x - self.gx_offset
        imu_msg.angular_velocity.y = msg.angular.y - self.gy_offset
        imu_msg.angular_velocity.z = msg.angular.z - self.gz_offset
        imu_msg.angular_velocity_covariance = self.angular_velocity_cov
        # # Accelerometer data in m/s^2
        imu_msg.linear_acceleration.x = msg.linear.x - self.linear_acceleration_x_offset
        imu_msg.linear_acceleration.y = msg.linear.y - self.linear_acceleration_y_offset
        imu_msg.linear_acceleration.z = msg.linear.z - self.linear_acceleration_z_offset
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_cov
        # # Publish the IMU data
        # self.get_logger().info(f'rpy: {self.rpy_cal(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z, imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z)}')
        self.imu_msg = imu_msg

    def wheel_vel_callback(self, msg: Twist):
        now = self.get_clock().now()
        dt = now - self.time_last
        self.time_last = now
        dt = dt.nanoseconds / NS_TO_SEC
        
        # Get the wheel velocities from the joint_states
        wl = msg.angular.x # left wheel velocity
        wr = msg.angular.z # right wheel velocity
        # self.get_logger().info(f'wl: {wl}, wr: {wr}')

        # calculate the linear and angular velocity of the robot
        vx = ((self._WHEEL_RADIUS/2) * (wl + wr) ) 
        wz = ((self._WHEEL_RADIUS/self._BASE_WIDTH) * (wr - wl))

        # Publish the wheel odometry to EKF
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        # Velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = wz

        # Position
        ds = vx * dt
        dtheta = wz * dt

        if vx != 0:
            # calculate distance traveled in x and y
            x = np.cos(dtheta) * ds
            y = -np.sin(dtheta) * ds
            # calculate the final position of the robot
            self.x += (np.cos(self.wz) * x - np.sin(self.wz) * y)
            self.y += (np.sin(self.wz) * x + np.cos(self.wz) * y)

        if wz != 0:
            self.wz += dtheta

        # If want to use pos on ekf
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Publish the odometry
        self.odom_msg = odom_msg
        # self.wheel_odom_pub.publish(odom_msg)


        # self.pose[0] = self.x
        # self.pose[1] = self.y
        # self.pose[2] = self.wz

        # pos0 = self.x
        # pos1 = self.y
        # pos2 = self.wz
        # ds = vx
        # dtheta = wz

        # self.get_logger().info(f'x: {pos0}, y: {pos1}, theta: {pos2}')
        # # Covariance matrix
        # # Fp = np.matrix([[1, 0, -ds*np.sin(self.pose[2])], [0, 1, ds*np.cos(self.pose[2])], [0, 0, 1]], dtype=float)
        # # Fdrl = np.matrix([[0.5*np.cos(self.pose[2]), 0.5*np.cos(self.pose[2])], [0.5*np.sin(self.pose[2]), 0.5*np.sin(self.pose[2])], [1/self._BASE_WIDTH, -1/self._BASE_WIDTH]], dtype=float)
        # # self.pose_cov = Fp * self.pose_cov * Fp.T + Fdrl * np.diag([self.linear_acceleration_cov[0], self.linear_acceleration_cov[4]]) * Fdrl.T

        # # self.get_logger().info(f'pose_cov: {self.pose_cov}')

        # # Publish the transform
        # odom_tf = TransformStamped()
        # odom_tf.header.stamp = now.to_msg()
        # odom_tf.header.frame_id = self.odom_frame_id
        # odom_tf.child_frame_id = self.base_frame_id
        # odom_tf.transform.translation.x = pos0
        # odom_tf.transform.translation.y = pos1
        # odom_tf.transform.translation.z = 0.0
        # q = Quaternion()
        # q.x = 0.0
        # q.y = 0.0
        # q.z = np.sin(pos2 / 2)
        # q.w = np.cos(pos2 / 2)
        # odom_tf.transform.rotation = q
        # # self.odom_broadcaster.sendTransform(odom_tf)


    def timer_callback(self):
        self.imu_pub.publish(self.imu_msg)
        self.wheel_odom_pub.publish(self.odom_msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = PMZBRosBridge()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
