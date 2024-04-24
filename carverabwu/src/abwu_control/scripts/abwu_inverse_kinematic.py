#!/usr/bin/python3

# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
import numpy as np
from std_msgs.msg import Float64MultiArray

class AbwuIKNode(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('AbwuIKNode')

        '''
        
        Stactic Parameters
        
        '''

        self._WHEEL_RADIUS = 0.075 # radius of the wheel
        self._BASE_WIDTH = 0.4   # distance between the wheels

        # Create a subscriber for the Diff Drive Publisher
        self.create_subscription(Twist, '/abwubot/cmd_vel', self.diff_drive_cont_sub_callback, 10)
        self.diff_drive_velocity = Twist()

        # Create a publisher for robot velocity commands
        self.velocity_cont_pub = self.create_publisher(Float64MultiArray, '/velocity_cont/commands', 10) # publish to /cmd_vel_Abwu topic
        self.cont_timer = 30
        self.create_timer(1 / self.cont_timer, self.velocity_cont_timer_callback)


        '''
        
        Set Node Status
        
        '''
        self.node_status = "ENABLED" # ENABLED, DISABLED

        self._node_name = "AbwuIKNode"

        self.get_logger().info(f'Abwu_inverse_kinemetic.py started with node name: {self._node_name}')


    def velocity_cont_timer_callback(self):
        if self.node_status == 'ENABLED':
            # Create a Float64MultiArray message to publish the velocity commands
            velocity_cont_msg = Float64MultiArray()
            velocity_cont_msg.data = self.velocity_controller_inverse_kinematics(self.diff_drive_velocity)
            # Publish the velocity commands
            self.velocity_cont_pub.publish(velocity_cont_msg)
        else:
            # Create a Float64MultiArray message to publish the velocity commands
            velocity_cont_msg = Float64MultiArray()
            velocity_cont_msg.data = [0.0, 0.0]
            # Publish the velocity commands
            self.velocity_cont_pub.publish(velocity_cont_msg)

    def diff_drive_cont_sub_callback(self, msg:Twist):
        # Create a Twist message to subscribe to the diff drive controller
        self.diff_drive_velocity.linear.x = msg.linear.x
        self.diff_drive_velocity.angular.z = msg.angular.z

    def velocity_controller_inverse_kinematics(self, diff_drive_velocity:Twist):
        # Create a Float64MultiArray message to publish the velocity commands
        vx = diff_drive_velocity.linear.x   # linear velocity
        w = diff_drive_velocity.angular.z   # angular velocity
        d = self._BASE_WIDTH
        r = self._WHEEL_RADIUS

        wl = (vx / r) - ((w * d) / (2 * r))
        wr = (vx / r) + ((w * d) / (2 * r))

        return [wl, wr]

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = AbwuIKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
