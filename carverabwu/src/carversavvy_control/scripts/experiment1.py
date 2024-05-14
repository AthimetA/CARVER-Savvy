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
from carversavvy_interfaces.srv import carversavvyBeginExperiment
import yaml
import pandas as pd
import os

NS_TO_SEC= 1000000000
RPM_TO_RAD = 0.10471975511965977
RAD_TO_RPM = 9.549296585513721
WHEEL_RADIUS = 0.034

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

        self.wheel_vel_sub = self.create_subscription(Twist, '/carversavvy_uros/wheel_vel', self.wheel_vel_callback, 10)
        self.wheel_vel_buffer = Twist()
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

        # Service
        self.srv = self.create_service(carversavvyBeginExperiment, 'carversavvy_experiment' , self.begin_calibration_callback)
        self.wheel_data = []
        self.begin_experiment = False
        self.target_rpm = 0
        self.n = 0
        self.hold = 0
        self._max_n = 250
        self.experiment_id = 0

        srv_text= 'ros2 service call /carversavvy_experiment carversavvy_interfaces/srv/carversavvyBeginExperiment "{id: 0, target_rpm: 0}"'
        self.get_logger().info(srv_text)

    def wheel_vel_callback(self, msg: Twist):
        self.wheel_vel_buffer = msg
        self.left_wheel_vel = msg.angular.x * RAD_TO_RPM
        self.right_wheel_vel = msg.angular.z * RAD_TO_RPM
        # self.get_logger().info(f'Received wheel velocity data: {self.left_wheel_vel}, {self.right_wheel_vel}')


    def cmd_timer_callback(self):
        now = self.get_clock().now()

        if self.begin_experiment == True and self.n < self._max_n:
            self.cmd_vel.linear.x = self.rpm_to_robot_vx(self.target_rpm)
            self.cmd_vel.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel)
            # self.get_logger().info(f'Published cmd_vel: vx={self.cmd_vel.linear.x}')

            time_now = (now.nanoseconds - self.start_time.nanoseconds) / NS_TO_SEC

            if self.hold >= 10:
                self.wheel_data.append([time_now, self.left_wheel_vel, self.right_wheel_vel])
                self.n += 1
                self.get_logger().info(f'Data Collected: {self.n}/{self._max_n}')
            else:
                self.hold += 1
                self.start_time = self.get_clock().now()

        elif self.begin_experiment == True and self.n == self._max_n:
            self.begin_experiment = False
            self.get_logger().info(f'Experiment Complete')

            # Write to yaml
            # self.get_logger().info(f'Writing to {self.imu_path}')
            self.wheel_data = np.array(self.wheel_data)
            data_frame = pd.DataFrame(self.wheel_data, columns=['time', 'left_wheel_vel', 'right_wheel_vel'])

            # Create a folder if it doesn't exist
            folder_name =  '/home/nuc11/CARVER-SAVVY/carverabwu/src/carversavvy_sensors/calibration'
            # Write the DataFrame to a CSV file inside the folder
            experiment_name = f'Experiment_1_{self.target_rpm}_id_{self.experiment_id}.csv'
            file_path = os.path.join(folder_name, experiment_name)
            data_frame.to_csv(file_path, index=False)

            print(f"DataFrame has been written to '{file_path}'.")

    def rpm_to_robot_vx(self,target_rpm):
        # calculate the linear and angular velocity of the robot
        wl = wr = target_rpm * RPM_TO_RAD
        vx = ((WHEEL_RADIUS/2) * (wl + wr) )
        return vx

    def begin_calibration_callback(self, request: carversavvyBeginExperiment.Request, response: carversavvyBeginExperiment.Response):
        self.get_logger().info(f'Begin experiment')
        self.n = 0
        self.experiment_id = request.id
        response.experiment_status = True
        self.target_rpm = request.target_rpm
        self.begin_experiment = True
        self.wheel_data = []
        self.hold = 0
        return response


# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = carversavvyExperimentTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
