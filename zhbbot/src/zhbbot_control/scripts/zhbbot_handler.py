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
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from zhbbot_interfaces.srv import RobotSentgoal, Goalreach , ZhbbotEnableNode, ZhbbotSendPath , ZhbbotUserSetgoal

class ZhbbotHandler(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('ZhbbotHandlerNode')

        # Create an action client for ComputePathToPose to get the path for the robot
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Create a service client for the ZhbbotSendPath service
        self.send_path_service_client = self.create_client(ZhbbotSendPath, '/zhbbot_service/send_path')

        # Create a service server for the ZhbbotUserSetgoal service
        self.user_setgoal_service = self.create_service(ZhbbotUserSetgoal,
                                                        '/zhbbot_service/user_setgoal',
                                                          self.user_setgoal_callback)
        
    def user_setgoal_callback(self, request: ZhbbotUserSetgoal.Request, response: ZhbbotUserSetgoal.Response):
        self.get_logger().info('User set goal request received')
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = request.x
        goal_pose.pose.position.y = request.y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = request.theta
        # Send the goal to the robot to compute the path from ComputePathToPose action server
        self.send_goal(goal_pose)
        response.status = "ZhbbotHandlerNode: Goal received from user"
        return response

    # Method to send a navigation goal to the ComputePathToPose action server
    def send_goal(self, pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    # Callback for handling the response from the ComputePathToPose action server
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return 
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    # Callback for handling the path result from the ComputePathToPose action server
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Path received from ComputePathToPose action server')

        # Send the path to the service
        self.send_path_service_call(result.path)

    def send_path_service_call(self, path):
        request = ZhbbotSendPath.Request()
        request.path = path
        while not self.send_path_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.send_path_service_client.call_async(request)
        future.add_done_callback(self.send_path_service_callback)

    def send_path_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Path sent to the service RECEIVER NODE RESPONSE: %s' % response.status)
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = ZhbbotHandler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
