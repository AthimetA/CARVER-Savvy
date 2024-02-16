#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import yaml
from math import sin, cos, pi
import numpy as np
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster

class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.pub_odom = self.create_publisher(Odometry,'/wheel/odom',10)
        
        # self.cmd_vel_subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.cmd_vel = Twist()
        self.period = 0.1
        
        self.counter = 0
        self.dt = 0.1

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.wheel = np.array([0.0,0.0])
        self.odom_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(self.period,self.timer_callback)
        # load yaml file
        # with open(sys.argv[1]) as f:
        #     model_parameter = yaml.load(f, Loader=yaml.loader.SafeLoader)
        self.wheel_separation = 0.45#float(model_parameter['wheel_separation'])  #0.45
        self.wheel_radius = 0.08#float(model_parameter['wheel_radius'])          #0.08
    def timer_callback(self):
        self.odometry_compute()

    def integrate(self, r, b):
        self.vx = (self.wheel[0]+self.wheel[1])*r/2
        self.vy = 0.0
        self.vth = (self.wheel[1]-self.wheel[0])*r/b
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
        delta_th = self.vth * self.dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        print(str(self.vx) + " " + str(self.vth) + " " + str(self.x) + " " + str(self.y))

    def odometry_compute(self):
        r = self.wheel_radius
        b = self.wheel_separation
        self.integrate(r,b)
        # current_time = self.get_clock().now()
        # odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.th)
        # self.odom_broadcaster.sendTransform((self.x, self.y, 0.),odom_quat,current_time,"base_footprint","odom")
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # self.odom_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # set the velocity
        odom.child_frame_id = "base_footprint"
        # odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # p_cov = np.array([0.00001]*36).reshape(6,6)

		# position covariance
		# p_cov[0:2,0:2] = self.P[0:2,0:2]
		# orientation covariance for Yaw
		# x and Yaw
		# p_cov[5,0] = p_cov[0,5] = self.P[2,0]
		# y and Yaw
		# p_cov[5,1] = p_cov[1,5] = self.P[2,1]
		# Yaw and Yaw
		# p_cov[5,5] = self.P[2,2]

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
        # print(odom)
        # publish the message
        self.pub_odom.publish(odom)

    def joint_state_callback(self, msg):
        self.wheel = np.array(msg.velocity)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
