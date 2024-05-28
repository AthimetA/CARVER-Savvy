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
from carversavvy_trajectory_generation import Trajectory
from std_msgs.msg import Float64

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
        self.mode = 2

        # Create a publisher to publish the velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/carversavvy_vel_ref', 10)
        self.hz = 30
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

        self.distance_pub = self.create_publisher(Float64, '/carversavvy_distance', 10)

        # Init Trahectory Profile
        if self.mode == 1:
            self.Jmax = 0.2
            self.Amax = 0.1
            self.Vmax = 0.2
            self.Dist = 1.0
            self.trajectory = Trajectory(self.Amax, self.Jmax, self.Vmax, self.Dist)
            self.trajectory.calculate_trajectory()
            t = np.arange(0, self.trajectory.T[6], self.time_period)
            self.QJ = np.zeros(t.shape)
            self.QA = np.zeros(t.shape)
            self.QV = np.zeros(t.shape)
            self.QVP = np.zeros(t.shape)
            self.QX = np.zeros(t.shape)
            for i in range(len(t)):
                self.QJ[i], self.QA[i], self.QV[i], self.QVP[i], self.QX[i] = self.trajectory.trajectory_evaluation(t[i])
            self.QV = self.QV.tolist()
            self.QX = self.QX.tolist()

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
            if self.timer_counter <= 2/self.time_period:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)
                dist = 0.0
                self.distance_pub.publish(Float64(data = dist))
            else:
                if self.loop_counter < len(self.QV):
                    self.cmd_vel.linear.x = self.QV[self.loop_counter]
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_vel)

                    self.distance_pub.publish(Float64(data=self.QX[self.loop_counter]))
                    self.loop_counter += 1

                else:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_vel)
                    self.distance_pub.publish(Float64(data=self.QX[self.loop_counter-1]))

                    self.mode = 3
                    self.timer_counter = 0
                    self.loop_counter = 0
                    
            

        elif mode == 2:
            self.timer_counter += 1
            if self.timer_counter <= 2/self.time_period:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)
            elif 2/self.time_period < self.timer_counter <= 7/self.time_period:
                self.cmd_vel.linear.x = 0.2
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)
            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)

                self.mode = 3
                self.timer_counter = 0
                self.loop_counter = 0

        elif mode == 3:
            if self.timer_counter <= 2/self.time_period:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)
            elif 2/self.time_period < self.timer_counter <= 12/self.time_period:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -(2*np.pi)/10
                self.cmd_pub.publish(self.cmd_vel)
            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)
            self.timer_counter += 1

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
