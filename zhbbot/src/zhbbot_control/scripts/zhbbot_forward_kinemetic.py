#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class ZhbbotFKNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('ZhbbotFKNode')

        self.node_name = 'ZhbbotFKNode'
        self.node_enabled = True

        self.joint_states_buffer = JointState()
        # Create a subscriber for the Joint State Publisher
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # Create a publisher for robot velocity commands
        self.odom_topic_name = 'zhbbot_wheel/odom'
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10) # publish to /odom topic

        # create timer_callback
        self.timer_hz = 10
        self.create_timer(1.0 / self.timer_hz, self.timer_callback)

        # init parameters
        self.x = 0.0
        self.y = 0.0
        self.wz = 0.0


        self._WHEEL_RADIUS = 0.075 # radius of the wheel
        self._BASE_WIDTH = 0.4   # distance between the wheels

    def timer_callback(self):
        if self.node_enabled:
            try:
                self.forward_kinematic_cal(self.joint_states_buffer)
            except:
                self.get_logger().error('Did not receive joint_states yet')

    def joint_states_callback(self, msg:JointState):
        # Create a Twist message to subscribe to the diff drive controller
        self.joint_states_buffer = msg

    def forward_kinematic_cal(self, joint_states:JointState):
        now = self.get_clock().now()

        # Create a Float64MultiArray message to publish the velocity commands
        # Get the wheel velocities from the joint_states
        velocities = np.array(joint_states.velocity).astype(np.float64)

        # Get the wheel velocities from the joint_states
        wl = velocities[0] # left wheel velocity
        wr = velocities[1] # right wheel velocity

        # calculate the linear and angular velocity of the robot
        vx = (self._WHEEL_RADIUS/2) * (wl + wr)
        wz = (self._WHEEL_RADIUS/self._BASE_WIDTH) * (wr - wl)

        if vx != 0:
            # calculate distance traveled in x and y
            x = np.cos(wz) * vx
            y = -np.sin(wz) * vx
            # calculate the final position of the robot
            self.x = self.x + (np.cos(self.wz) * x - np.sin(self.wz) * y)
            self.y = self.y + (np.sin(self.wz) * x + np.cos(self.wz) * y)
        if wz != 0:
            self.wz = self.wz + wz

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
        odom.child_frame_id = "base_footprint"
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
    node = ZhbbotFKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()