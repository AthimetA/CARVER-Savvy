#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
import tf_transformations
import math
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import sys 

class carversavvyTestNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__(node_name='carversavvyTestNode')

        # Get arg mode
        # if len(sys.argv) < 2:
        #     self.mode = 3
        # else:
        #     self.mode = int(sys.argv[1])
        self.mode = 1

        # Create a publisher to publish the velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/carversavvy_cmd_vel', 10)
        self.hz = 10
        self.time_period = 1/self.hz
        self.cmd_timer = self.create_timer(self.time_period, self.cmd_timer_callback)
        self.cmd_vel = Twist()
        self.timer_counter = 0
        self.loop_counter = 0

        self.wheel_vel_sub = self.create_subscription(Twist, '/carversavvy_wheel_vel', self.wheel_vel_callback, 10)
        self.wheel_vel_buffer = Twist()
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

        self.imu_sub = self.create_subscription(Imu, '/carversavvy_imu', self.imu_callback, 10)
        self.imu_buffer = Imu()

        # Create Ramp for Testing mode 1
        self.start_vel = 0.0
        self.cmd_vel.linear.x = self.start_vel

    def wheel_vel_callback(self, msg: Twist):
        self.wheel_vel_buffer = msg
        self.left_wheel_vel = msg.angular.x
        self.right_wheel_vel = msg.angular.z
        self.get_logger().info(f'Received wheel velocity data: {self.left_wheel_vel}, {self.right_wheel_vel}')

    def imu_callback(self, msg):
        self.imu_buffer = msg
        # self.get_logger().info(f'Received IMU data: ax={self.imu_buffer.linear_acceleration.x}, ay={self.imu_buffer.linear_acceleration.y}, az={self.imu_buffer.linear_acceleration.z}, gx={self.imu_buffer.angular_velocity.x}, gy={self.imu_buffer.angular_velocity.y}, gz={self.imu_buffer.angular_velocity.z}')
        # self.get_logger().info(f'Received IMU data: {self.imu_buffer}')

    def cmd_timer_callback(self):
        
        mode = self.mode

        if mode == 1:
            self.timer_counter += 1
            if self.timer_counter <= 3/self.time_period:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)

            elif 3/self.time_period < self.timer_counter <= 8/self.time_period:
                self.cmd_vel.linear.x += 0.2/(5/self.time_period)
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)

            elif 8/self.time_period < self.timer_counter <= 18/self.time_period:
                self.cmd_vel.linear.x = 0.2
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)

            elif 18/self.time_period < self.timer_counter <= 23/self.time_period:
                self.cmd_vel.linear.x -= 0.2/(5/self.time_period)
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)

            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)

        elif mode == 2:
            self.timer_counter += 1
            if self.timer_counter <= 10/self.time_period:
                self.cmd_vel.linear.x = 0.2
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)
            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)

        elif mode == 3:
            self.timer_counter += 1
            if self.timer_counter <= 10/self.time_period:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -(2*np.pi/4)/10
                self.cmd_pub.publish(self.cmd_vel)
            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)


        elif mode == 4:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel)

        


        self.get_logger().info(f'Published cmd_vel: vx={self.cmd_vel.linear.x}, wz={self.cmd_vel.angular.z}')


# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = carversavvyTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
