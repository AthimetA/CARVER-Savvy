#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist, Point
import tf_transformations
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Pose, TransformStamped, Quaternion
import numpy as np
from sensor_msgs.msg import LaserScan, JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from zhbbot_interfaces.srv import RobotSentgoal, Goalreach

# import all other neccesary libraries
import sys

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

        # Create a TransformBroadcaster
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)

        # init parameters
        self.x = 0.0
        self.y = 0.0
        self.wz = 0.0

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

        wl = velocities[0]
        wr = velocities[1]
        r = 0.075                           # radius of the wheel
        base_width = 0.4                             # distance between the wheels

        vx = (r/2) * (wl + wr)
        wz = (r/base_width) * (wr - wl)

        # d_left = joint_states.position[0]
        # d_right = joint_states.position[1]

        # # distance traveled is the average of the two wheels 
        # d = (d_left + d_right) / 2
        # # this approximation works (in radians) for small angles
        # th = (d_right - d_left) / base_width

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

        # transform_stamped_msg = TransformStamped()
        # transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        # transform_stamped_msg.header.frame_id = 'odom'
        # transform_stamped_msg.child_frame_id = 'base_footprint'
        # transform_stamped_msg.transform.translation.x = self.x
        # transform_stamped_msg.transform.translation.y = self.y
        # transform_stamped_msg.transform.translation.z = 0.0
        # transform_stamped_msg.transform.rotation.x = quaternion.x
        # transform_stamped_msg.transform.rotation.y = quaternion.y
        # transform_stamped_msg.transform.rotation.z = quaternion.z
        # transform_stamped_msg.transform.rotation.w = quaternion.w
        # self.odom_broadcaster.sendTransform(transform_stamped_msg)

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
        self.odom_pub.publish(odom)

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = ZhbbotFKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# #!/usr/bin/python3
# import sys
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3, TransformStamped
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState
# import yaml
# from math import sin, cos, pi
# import numpy as np
# from nav_msgs.msg import Odometry
# import tf_transformations
# from tf2_ros import TransformBroadcaster

# class WheelOdom(Node):
#     def __init__(self):
#         super().__init__('wheel_odometry')
#         self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
#         self.pub_odom = self.create_publisher(Odometry,'/zhbbot_wheel/odom',10)
        
#         # self.cmd_vel_subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
#         self.cmd_vel = Twist()
#         self.period = 0.1
        
#         self.counter = 0
#         self.dt = 0.1

#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0
#         self.vx = 0.0
#         self.vy = 0.0
#         self.vth = 0.0
#         self.wheel = np.array([0.0,0.0])
#         self.odom_broadcaster = TransformBroadcaster(self)

#         self.timer = self.create_timer(self.period,self.timer_callback)
#         # load yaml file
#         # with open(sys.argv[1]) as f:
#         #     model_parameter = yaml.load(f, Loader=yaml.loader.SafeLoader)
#         self.wheel_separation = 0.45#float(model_parameter['wheel_separation'])  #0.45
#         self.wheel_radius = 0.08#float(model_parameter['wheel_radius'])          #0.08
#     def timer_callback(self):
#         self.odometry_compute()

#     def integrate(self, r, b):
#         self.vx = (self.wheel[0]+self.wheel[1])*r/2
#         self.vy = 0.0
#         self.vth = (self.wheel[1]-self.wheel[0])*r/b
#         delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
#         delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
#         delta_th = self.vth * self.dt

#         self.x += delta_x
#         self.y += delta_y
#         self.th += delta_th

#         print(str(self.vx) + " " + str(self.vth) + " " + str(self.x) + " " + str(self.y))

#     def odometry_compute(self):
#         r = self.wheel_radius
#         b = self.wheel_separation
#         self.integrate(r,b)
#         # current_time = self.get_clock().now()
#         # odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.th)
#         # self.odom_broadcaster.sendTransform((self.x, self.y, 0.),odom_quat,current_time,"base_footprint","odom")
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_footprint'
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0
#         q = tf_transformations.quaternion_from_euler(0, 0, self.th)
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]
#         # self.odom_broadcaster.sendTransform(t)

#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = "odom"

#         # set the position
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation.x = q[0]
#         odom.pose.pose.orientation.y = q[1]
#         odom.pose.pose.orientation.z = q[2]
#         odom.pose.pose.orientation.w = q[3]

#         # set the velocity
#         odom.child_frame_id = "base_footprint"
#         # odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
#         odom.twist.twist.linear.x = self.vx
#         odom.twist.twist.linear.y = self.vy
#         odom.twist.twist.linear.z = 0.0

#         odom.twist.twist.angular.x = 0.0
#         odom.twist.twist.angular.y = 0.0
#         odom.twist.twist.angular.z = self.vth

#         # p_cov = np.array([0.00001]*36).reshape(6,6)

# 		# position covariance
# 		# p_cov[0:2,0:2] = self.P[0:2,0:2]
# 		# orientation covariance for Yaw
# 		# x and Yaw
# 		# p_cov[5,0] = p_cov[0,5] = self.P[2,0]
# 		# y and Yaw
# 		# p_cov[5,1] = p_cov[1,5] = self.P[2,1]
# 		# Yaw and Yaw
# 		# p_cov[5,5] = self.P[2,2]

#         odom.pose.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                  0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
#                                  0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
#                                  0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
#                                  0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
#                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
#         odom.twist.covariance = [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
#                                  0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
#                                  0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
#                                  0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
#                                  0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
#                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
#         # print(odom)
#         # publish the message
#         self.pub_odom.publish(odom)

#     def joint_state_callback(self, msg):
#         self.wheel = np.array(msg.velocity)


# def main(args=None):
#     rclpy.init(args=args)
#     node = WheelOdom()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__=='__main__':
#     main()
