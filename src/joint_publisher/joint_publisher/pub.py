#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback, 10)


        self.Robot_Length = 0.195
        self.Robot_Width = 0.235
        self.WheelRadius = 0.075

        # Define joint names
        self.joint_names = ['BL_joint', 'BR_joint', 'FL_joint','FR_joint' ]
        

    def listener_callback(self, msg:Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        fl = (self.vx - self.vy - (self.Robot_Length+self.Robot_Width)*self.wz) / self.WheelRadius
        fr = (self.vx + self.vy + (self.Robot_Length+self.Robot_Width)*self.wz) / self.WheelRadius
        rl = (self.vx + self.vy - (self.Robot_Length+self.Robot_Width)*self.wz) / self.WheelRadius
        rr = (self.vx - self.vy + (self.Robot_Length+self.Robot_Width)*self.wz) / self.WheelRadius

        msg.velocity = [bl, br, fl, fr] = [rl, rr, fl, fr] 
        

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

