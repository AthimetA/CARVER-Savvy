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

from zhbbot_interfaces.srv import RobotSentgoal, Goalreach, ZhbbotSendPath , ZhbbotUserSetgoal, ZhbbotSetNodeStaus

from nav_msgs.msg import Odometry

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
        
        '''
        
        Rosbot position and orientation

        '''

        self.odom_ekf_read = self.create_subscription(Odometry, '/odometry/local', self.odom_ekf_callback, 10)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        '''
        
        Actknowledged Timer

        '''
        self.actknowledged_period = 5.0
        self.last_actknowledged_timer = self.create_timer(self.actknowledged_period, self.actknowledged_callback)

        '''
        
        Handler timer
        
        '''

        self.handler_period = 0.1
        self.handler_timer = self.create_timer(self.handler_period, self.handler_callback)

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0

        self.error_range = 1.0

        '''
        
        Node status client

        '''
        # Node status
        self.node_status = "SLEEP"
        self.slave_node_name = ["ZhbbotVFFNode", "ZhbbotIKNode"]
        self.slave_node_status = {slave_node: "DISABLED" for slave_node in self.slave_node_name}
        self.node_status_client = {slave_node: self.create_client(ZhbbotSetNodeStaus,
                                                                   f'/zhbbot_service/{slave_node}/set_node_status') for slave_node in self.slave_node_name}

    def send_node_status(self, node_name, status):
        request = ZhbbotSetNodeStaus.Request()
        request.node_status = status
        while not self.node_status_client[node_name].wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.node_status_client[node_name].call_async(request)
        future.add_done_callback(self.node_status_callback)

    def node_status_callback(self, future):
        try:
            response = future.result()
            self.slave_node_status[response.node_name] = response.call_back_status
            text = f'Handler ---- > {response.node_name}, Node status: {response.call_back_status}'
            self.get_logger().info(text)
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

    def path_reach_check(self):
        if self._get_distance(self.robot_x, self.robot_y, self.goal_x, self.goal_y) < self.error_range:
            return True
        return False

    def handler_callback(self):
        if self.node_status == "ENABLED":
            # self.get_logger().info('Handler: Node is enabled')
            # self.get_logger().info('Handler: Path reach check result: %s' % self.path_reach_check())
            if self.path_reach_check():
                self.get_logger().info('Handler: Path reached')
                self.node_status = "SLEEP"
                self.get_logger().info('Handler: Node is disabled')
                self.send_node_status("ZhbbotVFFNode", "DISABLED")
                self.send_node_status("ZhbbotIKNode", "DISABLED")
                self.get_logger().info('Handler: All nodes are disabled')
    
    def actknowledged_callback(self):
        self.get_logger().info('-'*50)
        self.get_logger().info(f'ZhbbotHandlerNode: Robot position: x: {self.robot_x:2f}, y: {self.robot_y:2f}, theta: {self.robot_theta:2f}')
        for slave_node in self.slave_node_status:
            self.get_logger().info(f'Slave node: {slave_node}, status: {self.slave_node_status[slave_node]}')
        

    def odom_ekf_callback(self, msg:Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.robot_theta = euler[2]

        
    def user_setgoal_callback(self, request: ZhbbotUserSetgoal.Request, response: ZhbbotUserSetgoal.Response):
        self.get_logger().info('User set goal request received')
        self.goal_x = request.x
        self.goal_y = request.y
        self.goal_theta = request.theta
        # Generate the goal pose from the request
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = request.x
        goal_pose.pose.position.y = request.y
        goal_pose.pose.position.z = 0.0
        quaternion = tf_transformations.quaternion_from_euler(0, 0, request.theta)
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
        # Send the goal to the robot to compute the path from ComputePathToPose action server
        self.send_goal(goal_pose)
        response.status = "ZhbbotHandlerNode: Goal received from user"

        # Set the node status to "ENABLED"
        self.node_status = "ENABLED"

        self.send_node_status("ZhbbotVFFNode", "ENABLED")
        self.send_node_status("ZhbbotIKNode", "ENABLED")

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
            return None
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

    def _get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = ZhbbotHandler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
