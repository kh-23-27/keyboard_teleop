#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2025 Kenta Hirachi
# SPDX-License=Identifier: Apache 2.0

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class keyboard_receive(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber') 
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback,10)
        self.subscription  

    def listener_callback(self, msg):
        
        self.get_logger().info(f"Received Twist message: linear.x = {msg.linear.x}, linear.y = {msg.linear.y}, angular.z = {msg.angular.z}")

def main():
    rclpy.init()
    node = keyboard_receive()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

