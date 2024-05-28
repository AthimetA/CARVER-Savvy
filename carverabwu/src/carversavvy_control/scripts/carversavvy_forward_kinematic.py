#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import numpy as np
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import Pose
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import tf_transformations

NS_TO_SEC= 1000000000

class carversavvyFKNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('carversavvyFKNode')

        self._WHEEL_RADIUS = 0.050 # radius of the wheel
        self._BASE_WIDTH = 0.270   # distance between the wheels

        self.node_name = 'carversavvyFKNode'
        self.node_enabled = True

        self.wheelL_vel = Float32()
        self.wheelR_vel = Float32()
        # Create a subscriber for the Wheels Velocity
        self.create_subscription(Float32, '/wheelL_vel', self.wheelL_callback, 10)
        self.create_subscription(Float32, '/wheelR_vel', self.wheelR_callback, 10)

        # IMU subscriber
        self.imu_yaw_sub = self.create_subscription(Float32, '/IMU_yaw', self.imu_yaw_callback, 10)
        self.imu_ax_sub = self.create_subscription(Float32, '/IMU_ax', self.imu_ax_callback, 10)
        self.imu_vz_sub = self.create_subscription(Float32, '/IMU_vz', self.imu_vz_callback, 10)
        # Init IMU Data
        self.imu_yaw_buffer = 0.0
        self.imu_ax_buffer = 0.0
        self.imu_vz_buffer = 0.0

        # Read IMu Yaml File
        # Get the filepath to your config file
        imu_config_path = os.path.join(
            get_package_share_directory("carversavvy_sensors"), 
            'config','imu_calibration_result.yaml'
        )                       
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
                self.get_logger().info('Carversavvy initialized with IMU calibration data.')
        else:
            self.get_logger().info('Carversavvy initialized without IMU calibration data.')
            self.gx_offset = 0.0
            self.gy_offset = 0.0
            self.gz_offset = 0.0
            self.linear_acceleration_x_offset = 0.0
            self.linear_acceleration_y_offset = 0.0
            self.linear_acceleration_z_offset = 0.0
            self.linear_acceleration_cov = np.zeros(9)
            self.angular_velocity_cov = np.zeros(9)

        # Create a publisher for robot velocity commands
        self.odom_topic_name = 'carversavvy/wheel/odom'
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10) # publish to /odom topic

        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, '/carversavvy/imu_ros', 10)

        self.vx_cal_topic = 'carversavvy/vx_cal'
        self.vx_cal_pub = self.create_publisher(Float64, self.vx_cal_topic, 10) # publish to /odom topic

        # Create Publisher for the robot pose
        self.pose_topic_name = 'carversavvy/pose'
        self.pose_pub = self.create_publisher(Pose, self.pose_topic_name, 10)

        self.distance_pub = self.create_publisher(Float64, '/carversavvy_distance', 10)

        # create timer_callback
        self.timer_hz = 30
        self.create_timer(1.0 / self.timer_hz, self.timer_callback)

        # init parameters
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.time_last = self.get_clock().now()

    def wheelL_callback(self, msg:Float32):
        self.wheelL_vel = msg.data

    def wheelR_callback(self, msg:Float32):
        self.wheelR_vel = msg.data

    def imu_yaw_callback(self, msg:Float32):
        self.imu_yaw_buffer = msg.data
    
    def imu_ax_callback(self, msg:Float32):
        self.imu_ax_buffer = msg.data
    
    def imu_vz_callback(self, msg:Float32):
        self.imu_vz_buffer = msg.data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        # Gyroscope data in rad/s
        imu_msg.angular_velocity.x = self.imu_ax_buffer - self.gx_offset
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.angular_velocity_covariance = self.angular_velocity_cov
        # # Accelerometer data in m/s^2
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = self.imu_vz_buffer - self.linear_acceleration_z_offset
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_cov

        imu_msg.orientation.z = self.imu_yaw_buffer
        
        # Publish the IMU data
        self.imu_msg = imu_msg
        self.imu_pub.publish(imu_msg)

    def timer_callback(self):
        if self.node_enabled:
            try:
                self.forward_kinematic_cal(self.wheelL_vel, self.wheelR_vel)
            except:
                self.get_logger().error('Did not receive joint_states yet')
    
    def forward_kinematic_cal(self, wl, wr):
        now = self.get_clock().now()
        dt = now - self.time_last
        self.time_last = now
        dt = dt.nanoseconds / NS_TO_SEC

        if type(wl) == Float32:
            wl = wl.data
        if type(wr) == Float32:
            wr = wr.data

        # calculate the linear and angular velocity of the robot
        vx = (self._WHEEL_RADIUS/2) * (wl + wr)

        #publish the calculated vx
        vx_cal = Float64()
        vx_cal.data = vx
        self.vx_cal_pub.publish(vx_cal)

        wz = (self._WHEEL_RADIUS/self._BASE_WIDTH) * (wr - wl)

        # Position
        ds = vx * dt
        dtheta = wz * dt

        if wz != 0:
            self.theta += dtheta

        if vx != 0:
            # calculate distance traveled in x and y
            x = np.cos(dtheta) * ds
            y = -np.sin(dtheta) * ds
            # calculate the final position of the robot
            self.x += ((np.cos(self.theta) * x) - (np.sin(self.theta) * y))
            self.y += ((np.sin(self.theta) * x) + (np.cos(self.theta) * y))

        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw angle to quaternion
        qx,qy,qz,qw = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set the velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = wz
        
        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = carversavvyFKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()