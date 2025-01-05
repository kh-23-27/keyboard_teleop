#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2025 Kenta Hirachi
# SPDX-License-Identifier: Apache 2.0

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


instructions = """
---------------------------
Moving around:

   q    w    e
   a         d
   z    s    c

j: Increase speed
k: Decrease speed
l: Reset speed to 0.5
---------------------------
"""


key_mapping = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, -1),
    'd': (0, 1),
    'q': (1, -1),
    'e': (1, 1),
    'z': (-1, -1),
    'c': (-1, 1),
}

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.5
        self.twist = Twist()
        print(instructions)

    def get_key(self):

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return None

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in key_mapping:
                    self.twist.linear.x = self.speed * key_mapping[key][0]
                    self.twist.linear.y = self.speed * key_mapping[key][1]
                elif key == 'j':  
                    self.speed += 0.1
                    print(f"Speed increased to {self.speed}")
                elif key == 'k':  
                    self.speed = max(0.1, self.speed - 0.1)
                    print(f"Speed decreased to {self.speed}")
                elif key == 'l':  #
                    self.speed = 0.5
                    print("Speed reset to 0.5")
                elif key == '\x03':  
                    break
                else:  
                    self.twist.linear.x = 0.0
                    self.twist.linear.y = 0.0

                self.publisher.publish(self.twist)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.publisher.publish(self.twist)
            print("\nExiting teleop_twist_keyboard...")

def main():
    rclpy.init()
    node = TeleopTwistKeyboard()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

