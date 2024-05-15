#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

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

        self.wheelL_vel = Float64()
        self.wheelR_vel = Float64()
        # Create a subscriber for the Wheels Velocity
        self.create_subscription(Float64, '/wheelL_vel', self.wheelL_callback, 10)
        self.create_subscription(Float64, '/wheelR_vel', self.wheelR_callback, 10)

        # Create a publisher for robot velocity commands
        self.odom_topic_name = 'carversavvy/wheel/odom'
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10) # publish to /odom topic

        self.vx_cal_topic = 'vx_cal'
        self.vx_cal_pub = self.create_publisher(Float64, self.vx_cal_topic, 10) # publish to /odom topic

        # create timer_callback
        self.timer_hz = 10
        self.create_timer(1.0 / self.timer_hz, self.timer_callback)

        # init parameters
        self.x = 0.0
        self.y = 0.0
        self.wz = 0.0
        self.time_last = self.get_clock().now()

    def timer_callback(self):
        if self.node_enabled:
            try:
                self.forward_kinematic_cal(self.wheelL_vel, self.wheelR_vel)
            except:
                self.get_logger().error('Did not receive joint_states yet')


        # self.forward_kinematic_cal(self.wheelL_vel, self.wheelR_vel)

    def wheelL_callback(self, msg:Float64):
        self.wheelL_vel = msg.data

    def wheelR_callback(self, msg:Float64):
        self.wheelR_vel = msg.data

    def forward_kinematic_cal(self, wl, wr):
        now = self.get_clock().now()
        dt = now - self.time_last
        self.time_last = now
        dt = dt.nanoseconds / NS_TO_SEC

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

        if vx != 0:
            # calculate distance traveled in x and y
            x = np.cos(dtheta) * ds
            y = -np.sin(dtheta) * ds
            # calculate the final position of the robot
            self.x += (np.cos(self.wz) * x - np.sin(self.wz) * y)
            self.y += (np.sin(self.wz) * x + np.cos(self.wz) * y)

        if wz != 0:
            self.wz += dtheta

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(self.wz / 2)
        quaternion.w = np.cos(self.wz / 2)

        # Create an Odometry message to publish the odometry information
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz
        odom.pose.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        odom.twist.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        self.odom_pub.publish(odom)

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = carversavvyFKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()