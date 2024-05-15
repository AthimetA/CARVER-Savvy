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
from carversavvy_interfaces.srv import carversavvyBeginOdomExperiment
import yaml
import pandas as pd
import os

NS_TO_SEC= 1000000000
WHEEL_RADIUS = 0.034
TOTAL_DISTANT = 2.0 # m
ROBOT_VEL = 0.1 # m/s

class carversavvyExperimentTestNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__(node_name='carversavvyTestNode')

        # Create a publisher to publish the velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/carversavvy_cmd_vel', 10)
        self.hz = 5
        self.time_period = 1/self.hz
        self.cmd_timer = self.create_timer(self.time_period, self.cmd_timer_callback)
        self.cmd_vel = Twist()
        self.timer_counter = 0
        self.loop_counter = 0
        self.time_last = self.get_clock().now()
        self.start_time = self.get_clock().now()

        # Wheel Odometry
        self.wheel_odom_sub = self.create_subscription(Odometry, '/carversavvy_ros/wheel_odom', self.wheel_odom_callback, 10)
        self.wheel_odom_buffer = Odometry()

        # Odometry filter
        self.odom_filter_sub = self.create_subscription(Odometry, '/carversavvy_ros/odometry', self.odom_filter_callback, 10)
        self.odom_filter_buffer = Odometry()

        # Service
        self.srv = self.create_service(carversavvyBeginOdomExperiment, '/carversavvy_experiment_odom' , self.begin_calibration_callback)
        self.total_time = TOTAL_DISTANT / ROBOT_VEL # s
        self.total_time_steps = np.arange(0, self.total_time + self.time_period, self.time_period)
        self.true_x = ROBOT_VEL * self.total_time_steps
        self.true_y = np.zeros(self.total_time_steps.shape)

        self.begin_experiment = False
        self.n = 0
        self.hold = 0
        self._max_n = len(self.total_time_steps)
        self.experiment_id = 0
        self.save_data_list = []
        self.time_flag = True

        srv_text= 'ros2 service call /carversavvy_experiment_odom carversavvy_interfaces/srv/carversavvyBeginOdomExperiment "{id: 0}"'
        self.get_logger().info(srv_text)

    def begin_calibration_callback(self, request: carversavvyBeginOdomExperiment.Request, response: carversavvyBeginOdomExperiment.Response):
        self.get_logger().info(f'Begin experiment')
        # Clear the parameters
        self.n = 0
        self.experiment_id = request.id
        self.begin_experiment = True
        self.hold = 0
        self.time_flag = True

        # Clear the data
        self.save_data_list = []

        response.experiment_status = True
        
        return response

    
    def wheel_odom_callback(self, msg: Odometry):
        self.wheel_odom_buffer = msg
        # self.get_logger().info(f'Wheel Odometry: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}')

    def odom_filter_callback(self, msg):
        self.odom_filter_buffer = msg
        # self.get_logger().info(f'Odometry Filter: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}')

    def cmd_timer_callback(self):
        if self.begin_experiment == True and self.n < self._max_n:
            # self.get_logger().info(f'Published cmd_vel: vx={self.cmd_vel.linear.x}')

            if self.hold >= 10:
                now = self.get_clock().now()
                if self.time_flag == True:
                    self.start_time = self.get_clock().now()
                    self.time_flag = False
                
                time_now = (now.nanoseconds - self.start_time.nanoseconds) / NS_TO_SEC
                
                self.cmd_vel.linear.x = ROBOT_VEL
                self.cmd_vel.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_vel)
                self.save_data_list.append([time_now, self.wheel_odom_buffer.pose.pose.position.x, self.wheel_odom_buffer.pose.pose.position.y, self.odom_filter_buffer.pose.pose.position.x, self.odom_filter_buffer.pose.pose.position.y])
                self.n += 1
                self.get_logger().info(f'Data Collected: {self.n}/{self._max_n}')
            else:
                self.hold += 1
                

        elif self.begin_experiment == True and self.n == self._max_n:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel)
            
            self.begin_experiment = False
            self.get_logger().info(f'Experiment Complete')

            # Write to yaml
            # self.get_logger().info(f'Writing to {self.imu_path}')
            now = self.get_clock().now()
            time_now = (now.nanoseconds - self.start_time.nanoseconds) / NS_TO_SEC
            self.save_data_list.append([time_now, self.wheel_odom_buffer.pose.pose.position.x, self.wheel_odom_buffer.pose.pose.position.y, self.odom_filter_buffer.pose.pose.position.x, self.odom_filter_buffer.pose.pose.position.y])

            save_data = np.array(self.save_data_list)
            data_frame = pd.DataFrame(save_data, columns=['time', 'wheel_odom_x', 'wheel_odom_y', 'odom_filter_x', 'odom_filter_y'])
            # Add one more time step to the true_x and true_y'
            data_length = len(self.save_data_list)
            len_true_x = len(self.true_x)
            nloop = 1
            while len_true_x < data_length:
                self.true_x = np.append(self.true_x, self.true_x[-1] + self.time_period * ROBOT_VEL * nloop)
                self.true_y = np.append(self.true_y, self.true_y[-1] + 0.0)
                len_true_x += 1
                nloop += 1
            data_frame['true_x'] = self.true_x
            data_frame['true_y'] = self.true_y


            # Create a folder if it doesn't exist
            folder_name =  '/home/nuc11/CARVER-SAVVY/carvercarversavvy/src/carversavvy_sensors/calibration'
            # Write the DataFrame to a CSV file inside the folder
            experiment_name = f'Experiment_2_id_{self.experiment_id}.csv'
            file_path = os.path.join(folder_name, experiment_name)
            data_frame.to_csv(file_path, index=False)

            print(f"DataFrame has been written to '{file_path}'.")

        else:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel)
        


# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = carversavvyExperimentTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
