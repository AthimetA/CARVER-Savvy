#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry, Path
import tf_transformations
import math
import numpy as np
import sys

from zhbbot_interfaces.srv import ZhbbotSendPath , ZhbbotUserSetgoal, ZhbbotSetNodeStaus
from awbu_interfaces.srv import DrlStep, EnvReady, ObstacleStart ,ScoreStep

from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap

from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, HistoryPolicy

import time

from drlutils_test_graph import Test_Graph
import os

from std_msgs.msg import Float32MultiArray


UNKNOWN = 0
SUCCESS = 1
COLLISION = 2
TIMEOUT = 3
TUMBLE = 4

TOPIC_SCAN = '/scan'
NUM_SCAN_SAMPLES = 180
LIDAR_DISTANCE_CAP = 12.0

GRAPH_DRAW_INTERVAL = 5
EPISODE_TEST = 1000

TOPIC_CLOCK = '/clock'
from rosgraph_msgs.msg import Clock
EPISODE_TIMEOUT_SECONDS = 20.0

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ZhbbotHandler(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('ZhbbotHandlerNode')

        qos = QoSProfile(depth=10)
        qos_clock = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        # self.step_score_client = self.create_client(ScoreStep, '/nav2/score')
        self.scan_sub = self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.scan_ranges = np.zeros(NUM_SCAN_SAMPLES)
        self.obstacle_distance_nearest = LIDAR_DISTANCE_CAP

        self.clock_sub = self.create_subscription(Clock, TOPIC_CLOCK, self.clock_callback, qos_profile=qos_clock)
        # --------------- Time and Episode --------------- #
        self.episode_timeout = EPISODE_TIMEOUT_SECONDS
        self.time_sec = 0
        self.episode_start_time = 0
        # Episode variables
        self.episode_deadline = np.inf
        self.reset_deadline = False

        # Create an action client for ComputePathToPose to get the path for the robot
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Create a service client for the ZhbbotSendPath service
        self.send_path_service_client = self.create_client(ZhbbotSendPath, '/zhbbot_service/send_path')

        # Publisher for ik controller
        self.velocity_publisher_ik = self.create_publisher(Twist, '/diff_drive_zhbbot', 10)

        # Create a service server for the ZhbbotUserSetgoal service
        self.user_setgoal_service = self.create_service(ZhbbotUserSetgoal,
                                                        '/zhbbot_service/user_setgoal',
                                                          self.user_setgoal_callback)
        
        # Gazibo reset simulation client
        self.reset_simulation_client    = self.create_client(Empty, '/reset_world')
        self.gazebo_pause = self.create_client(Empty, '/pause_physics')
        self.gazebo_unpause = self.create_client(Empty, '/unpause_physics')

        # Amcl initial pose publisher
        self.amcl_initial_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Clear costmap service
        self.clear_costmap_client = self.create_client(ClearEntireCostmap, '/callglobal_costmap/clear_entirely_global_costmap')

        self.obstacle_start_client      = self.create_client(ObstacleStart, '/obstacle_start')
        
        '''
        
        Rosbot position and orientation

        '''

        self.odom_ekf_read = self.create_subscription(Odometry, '/odom_groud_truth_pose', self.odom_ekf_callback, 10)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_vw = 0.0

        self.odom_true_read = self.create_subscription(Odometry, '/odom_groud_truth_pose', self.odom_true_callback, 10)
        self.true_robot_x = 0.0
        self.true_robot_y = 0.0
        self.true_robot_theta = 0.0

        '''
        
        Actknowledged Timer

        '''
        self.actknowledged_period = 1.0
        self.last_actknowledged_timer = self.create_timer(self.actknowledged_period, self.actknowledged_callback)

        '''
        
        Handler timer
        
        '''

        self.handler_period = 0.01
        self.handler_timer = self.create_timer(self.handler_period, self.handler_callback)

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.__GOAL_THRESHOLD = 1.0

        self.error_range = 0.1

        '''
        
        Node status client

        '''
        # Node status
        self.node_status = "SLEEP"

        # Read the local planner from the parameter yaml file
        if len(sys.argv)>=2: 
            self.selected_local_planner = sys.argv[1]
        else:
            # Default local planner
            self.selected_local_planner = "ZhbbotVFFNode"

        self.get_logger().info(f'Handler: Selected local planner: {self.selected_local_planner}')

        self.slave_node_name = ["ZhbbotIKNode", self.selected_local_planner]
        self.slave_node_status = {slave_node: "DISABLED" for slave_node in self.slave_node_name}
        self.node_status_client = {slave_node: self.create_client(ZhbbotSetNodeStaus,
                                                                   f'/zhbbot_service/{slave_node}/set_node_status') for slave_node in self.slave_node_name}
    
        
        self.timeloop_timer = self.create_timer(0.1, self.timeloop_callback)
        # self.timelooper_get_score_timer = self.create_timer(0.1, self.timeloop_score_callback)
        self.loop_status = "SLEEP" # SLEEP, INIT, PROCESS, DONE
        self.loop_send_cmd = True
        self._EP_status = UNKNOWN

        self.local_ep = 0
        _path = '~'
        self.session_dir = os.path.join(_path, "nav2")
        if not os.path.exists(self.session_dir):
            os.makedirs(self.session_dir)
        self.test_graph = Test_Graph(session_dir=self.session_dir, first_episode=self.local_ep , continue_graph=True)

        self.sub_score = self.create_subscription(Float32MultiArray,'/score',self.score_callback,10)
        self.time_data = None
        self._get = True

    def scan_callback(self, msg: LaserScan):
        if len(msg.ranges) != NUM_SCAN_SAMPLES:
            print(f"more or less scans than expected! check model.sdf, got: {len(msg.ranges)}, expected: {NUM_SCAN_SAMPLES}")
        self.scan_ranges = np.array(msg.ranges)
        # normalize laser values
        self.obstacle_distance_nearest = LIDAR_DISTANCE_CAP
        for i in range(NUM_SCAN_SAMPLES):
            # Check for obstacles
            if self.scan_ranges[i] < self.obstacle_distance_nearest:
                self.obstacle_distance_nearest = self.scan_ranges[i]
        

    def clock_callback(self, msg: Clock):
        # Get current time
        self.time_sec = msg.clock.sec
        # Reset episode deadline
        if not self.reset_deadline:
            return
        # Reset episode deadline
        episode_time = self.episode_timeout
        # Set deadline
        self.episode_deadline = self.time_sec + episode_time
        # Reset variables
        self.reset_deadline = False

    def score_callback(self , msg):
        self.time_data = list(msg.data)

    def timeloop_callback(self):

        if self.loop_status == "INIT":
            
            self.start_protocall()

            self.loop_status = "PROCESS"

        elif self.loop_status == "PROCESS":

            self.episode_check()

        elif self.loop_status == "DONE":

            self.reset_protocall()

            self.get_logger().info(bcolors.OKGREEN + f'Episode status: {self._EP_status}' + bcolors.ENDC)

            self.loop_status = "INIT"

    def episode_check(self):
        
        THREHSOLD_GOAL = 1.2
        THRESHOLD_COLLISION = 0.6 # meters

        # Success
        if self._get_distance(self.robot_x, self.robot_y, self.goal_x, self.goal_y) < THREHSOLD_GOAL:
            self.get_logger().info(bcolors.OKGREEN + "Episode done, Agent reached the goal!" + bcolors.ENDC)
            self._EP_status = SUCCESS
            
        # Timeout
        elif self.time_sec >= self.episode_deadline:
            self.get_logger().info(bcolors.WARNING + "Episode done, Agent reached the timeout!" + bcolors.ENDC)
            self._EP_status = TIMEOUT
        # Collision
        elif self.obstacle_distance_nearest < THRESHOLD_COLLISION:
            self.get_logger().info(bcolors.FAIL + f"Episode done, Collision with obstacle: {self.obstacle_distance_nearest:.2f}" + bcolors.ENDC)
            self._EP_status = COLLISION

        if self._EP_status is not UNKNOWN:

            # Reset the episode deadline
            self.episode_deadline = np.inf

            # Reset the loop status
            self.loop_status = "DONE"

        if self._EP_status == SUCCESS or self._EP_status == TIMEOUT or self._EP_status == COLLISION:
            if self._get == True : 
                k_time , m_time, total_time= self.time_data
                total_time -= 3
                __text = f"k_step : {k_time} , m_time : {m_time} , total_time : {total_time} "
                self.get_logger().info(bcolors.OKBLUE + __text + bcolors.ENDC)

                self.local_ep  +=1 
                self.test_graph.update_data(0, self.local_ep, self._EP_status, k_time, m_time, total_time)


                if (self.local_ep  % EPISODE_TEST == 0):
                    self.test_graph.draw_plots(self.local_ep, save=True)
                    self.get_logger().info(bcolors.OKGREEN + f"Test Graph Drawn at Episode: {self.local_ep}" + bcolors.ENDC)
                    # Terminate the process
                    quit()
                elif (self.local_ep % GRAPH_DRAW_INTERVAL == 0) or (self.local_ep == 1):
                    self.test_graph.draw_plots(self.local_ep, save=False)
                self._get = False


    def start_protocall(self):

        self.goal_x = 15.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        # Send the goal to the robot to compute the path from ComputePathToPose action server
        self.send_goal([self.goal_x, self.goal_y, self.goal_theta])

        # Set the node status to "ENABLED"
        self.node_status = "ENABLED"

        self.send_node_status(self.selected_local_planner, "ENABLED")
        self.send_node_status("ZhbbotIKNode", "ENABLED")

        self.obstacle_start()

        # Set the episode start time
        self.reset_deadline = True
        self.episode_start_time = self.time_sec
        self._EP_status = UNKNOWN


    def reset_protocall(self):

        self._get = True
        # Reset the simulation
        self.reset_simulation()

        # Clear costmap
        self.clear_costmap()

        # Set amcl initial pose
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"

        self.amcl_initial_pose.publish(initial_pose)

        # Publish the zero velocity to the robot
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher_ik.publish(vel_msg)

        # Publish the goal reached status
        self.get_logger().info('Handler: Goal reached, Handler is SLEEP')
        self.send_node_status("ZhbbotIKNode", "DISABLED")
        self.send_node_status(self.selected_local_planner, "DISABLED")

        time.sleep(3)

    
    def obstacle_start(self):
        req = ObstacleStart.Request()
        while not self.obstacle_start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.obstacle_start_client.call_async(req)
    
    def clear_costmap(self):
        # while not self.clear_costmap_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('clear costmap service not available, waiting again...')
        self.clear_costmap_client.call_async(ClearEntireCostmap.Request())

    def reset_simulation(self):
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')
        self.reset_simulation_client.call_async(Empty.Request())

    def pause_simulation(self):
        # Pause simulation
        while not self.gazebo_pause.wait_for_service():
            self.get_logger().info('pause gazebo service not available, waiting again...')
        future = self.gazebo_pause.call_async(Empty.Request())
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                return None
            
    def unpause_simulation(self):
        # Unpause simulation
        while not self.gazebo_unpause.wait_for_service():
            self.get_logger().info('unpause gazebo service not available, waiting again...')
        future = self.gazebo_unpause.call_async(Empty.Request())
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                return None


    def send_node_status(self, node_name, status):
        request = ZhbbotSetNodeStaus.Request()
        request.node_status = status
        # while not self.node_status_client[node_name].wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again... local planner: %s' % self.selected_local_planner)
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
            if self.path_reach_check():
                self.get_logger().info('='*50)
                self.node_status = "SLEEP"
                self.get_logger().info('Handler: Path reached, Handler is SLEEP')
                self.send_node_status("ZhbbotIKNode", "DISABLED")
                self.send_node_status(self.selected_local_planner, "DISABLED")
                self.get_logger().info('Handler: All Slave nodes are DISABLED')
                self.get_logger().info('='*50)
    
    def actknowledged_callback(self):
        self.get_logger().info('-'*50)
        self.get_logger().info(f'ZhbbotHandlerNode: Robot position: x: {self.robot_x:2f}, y: {self.robot_y:2f}, theta: {self.robot_theta:2f} velx: {self.robot_vx:2f}, vely: {self.robot_vy:2f}, velw: {self.robot_vw:2f}')
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

        self.robot_vx = msg.twist.twist.linear.x
        self.robot_vy = msg.twist.twist.linear.y
        self.robot_vw = msg.twist.twist.angular.z

    def odom_true_callback(self, msg:Odometry):
        self.true_robot_x = msg.pose.pose.position.x
        self.true_robot_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.true_robot_theta = euler[2]

        
    def user_setgoal_callback(self, request: ZhbbotUserSetgoal.Request, response: ZhbbotUserSetgoal.Response):
        # Reset the simulation

        # self.reset_simulation()

        # # Clear costmap
        # self.clear_costmap()

        # # Set amcl initial pose
        # initial_pose = PoseWithCovarianceStamped()
        # initial_pose.header.frame_id = "map"

        # self.amcl_initial_pose.publish(initial_pose)

        # time.sleep(1)

        # self.get_logger().info('User set goal request received')
        # self.goal_x = request.x
        # self.goal_y = request.y
        # self.goal_theta = request.theta
        # # Send the goal to the robot to compute the path from ComputePathToPose action server
        # self.send_goal([self.goal_x, self.goal_y, self.goal_theta])

        # # Set the node status to "ENABLED"
        # self.node_status = "ENABLED"

        # self.send_node_status(self.selected_local_planner, "ENABLED")
        # self.send_node_status("ZhbbotIKNode", "ENABLED")

        response.status = "ZhbbotHandlerNode: Goal received from user"

        self.loop_status = "INIT"
        self.loop_send_cmd = True

        return response

    # Method to send a navigation goal to the ComputePathToPose action server
    def send_goal(self, gp):

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = gp[0]
        goal_pose.pose.position.y = gp[1]
        theta = gp[2] * (math.pi / 180.0)
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        star_pos = PoseStamped()
        star_pos.header.frame_id = "map"
        star_pos.pose.position.x = self.robot_x
        star_pos.pose.position.y = self.robot_y
        star_pos.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.robot_theta)
        star_pos.pose.orientation.x = q[0]
        star_pos.pose.orientation.y = q[1]
        star_pos.pose.orientation.z = q[2]
        star_pos.pose.orientation.w = q[3]
        

        self.get_logger().info('Sending goal to ComputePathToPose action server')
        self.get_logger().info(f'Goal pose: {goal_pose}')
        self.get_logger().info(f'Start pose: {star_pos}')

        CPTP = ComputePathToPose.Goal()
        CPTP.goal = goal_pose
        CPTP.start = star_pos
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(CPTP)
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
    # node.test_protocall()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
