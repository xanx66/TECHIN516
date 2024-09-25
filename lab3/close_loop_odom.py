#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class OdomCloseLoop(Node):
    def __init__(self):
        super().__init__('close_loop_odom')
        # debug mode
        self.debug = True
        self.sub = # TODO: subscribe to /odom topic
        # Publisher to control the velocity
        self.pub = # TODO: add the publisher to send velocity values
        self.position_x_list = []
        self.motion_move = Twist()
        self.motion_stop = Twist()
        # Set constant speed to move forward
        self.motion_move.linear.x = 0.15
        # Set speed to stop
        self.motion_stop.linear.x = 0.0

    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        self.position_x_list.append(position_x)
        # Ensure the list has at least 2 elements
        if len(self.position_x_list) > 2:
            # Check if the last recorded position is greater than the first one
            # plus the desired distance to travel
            if # TODO: fill in the condition to check whether the last item on the list
            # is greater than the first one plus the desired distance to be traveled
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Reached goal, stopping...")
                rclpy.shutdown()
            else:
                self.pub.publish(self.motion_move)
                if self.debug:
                    self.get_logger().info(f"Current position: {msg.pose.pose}")

def main(args=None):
    rclpy.init(args=args)
    odom_cl = OdomCloseLoop()
    rclpy.spin(odom_cl)
    odom_cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
