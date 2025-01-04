#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2024 Kenta Hirachi
# SPDX-License=Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Twist メッセージ型をインポート
from sshkeyboard import listen_keyboard

class KeyboardTeleop(Node):
    def __init__(self):  # コンストラクタ
        super().__init__('keyboard_teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.speed = 0.5
        self.pressed_keys = set()  # 押されているキーを管理するセット

        self.display_instructions()

    def display_instructions(self):
        msg = """
        ---------------------------
        Moving around:
             
         q              e
                w
             
            a       d

                s
         z              c

        j: Increase x, y speed
        k: Decrease x, y speed
        
        ---------------------------
        """
        self.get_logger().info(msg)

    def process_keys(self):
        
        if 'q' in self.pressed_keys:  # 左上
            self.twist.linear.x = self.speed
            self.twist.linear.y = -self.speed
        elif 'e' in self.pressed_keys: # 右上
            self.twist.linear.x = self.speed
            self.twist.linear.y = self.speed
        elif 'z' in self.pressed_keys:  # 左下
            self.twist.linear.x = -self.speed
            self.twist.linear.y = -self.speed
        elif 'c' in self.pressed_keys:  # 右下
            self.twist.linear.x = -self.speed
            self.twist.linear.y = self.speed
        elif 'w' in self.pressed_keys:  # 前進
            self.twist.linear.x = self.speed
            self.twist.linear.y = 0.0
        elif 's' in self.pressed_keys:  # 後退
            self.twist.linear.x = -self.speed
            self.twist.linear.y = 0.0
        elif 'a' in self.pressed_keys:  # 左移動
            self.twist.linear.x = 0.0
            self.twist.linear.y = -self.speed
        elif 'd' in self.pressed_keys:  # 右移動
            self.twist.linear.x = 0.0
            self.twist.linear.y = self.speed
        else:  # 停止
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0

        # メッセージをパブリッシュ
        self.publisher.publish(self.twist)

    def handle_key_press(self, key):
        # キー押下時の処理
        self.pressed_keys.add(key)
        if key == 'j':  # 速度アップ
            self.speed += 0.1
        elif key == 'k':  # 速度ダウン
            self.speed = max(0.1, self.speed - 0.1)  # 速度が0未満にならないようにする
        elif key == ' ':  # スペースで速度リセット
            self.speed = 0.5
        self.process_keys()

    def handle_key_release(self, key):
        # キー解放時の処理
        self.pressed_keys.discard(key)
        self.process_keys()

def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        # sshkeyboardのリスナーを開始
        listen_keyboard(
            on_press=node.handle_key_press,
            on_release=node.handle_key_release,
        )
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

