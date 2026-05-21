#!/usr/bin/env python3
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PrettyTeleop(Node):
    def __init__(self):
        super().__init__('pretty_master_wheel_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.20
        self.turn = 0.70
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.wheel_base = 0.32
        self.speed_scale = 300.0
        self.max_speed_cmd = 300
        self.min_effective_cmd = 80
        self.last_key = '-'
        self.last_publish = 0.0
        self.msg_count = 0
        self.last_motion_key_time = 0.0
        self.hold_timeout = 0.35
        self.auto_stop = False

    def clamp(self, value):
        return max(-self.max_speed_cmd, min(self.max_speed_cmd, int(round(value))))

    def min_effective(self, value):
        if value == 0:
            return 0
        if abs(value) < self.min_effective_cmd:
            return self.min_effective_cmd if value > 0 else -self.min_effective_cmd
        return value

    def wheel_cmds(self):
        right_mps = self.linear_x + self.angular_z * self.wheel_base / 2.0
        left_mps = self.linear_x - self.angular_z * self.wheel_base / 2.0
        right = self.min_effective(self.clamp(right_mps * self.speed_scale))
        left = self.min_effective(self.clamp(left_mps * self.speed_scale))
        return right, left

    def state(self):
        if abs(self.linear_x) < 0.01 and abs(self.angular_z) < 0.01:
            return 'STOP'
        if abs(self.linear_x) >= 0.01 and abs(self.angular_z) < 0.01:
            return 'FORWARD' if self.linear_x > 0 else 'BACKWARD'
        if abs(self.angular_z) >= 0.01 and abs(self.linear_x) < 0.01:
            return 'TURN_LEFT' if self.angular_z > 0 else 'TURN_RIGHT'
        if self.linear_x > 0:
            return 'FORWARD_LEFT' if self.angular_z > 0 else 'FORWARD_RIGHT'
        return 'BACKWARD_LEFT' if self.angular_z > 0 else 'BACKWARD_RIGHT'

    def handle_key(self, ch):
        self.last_key = repr(ch)[1:-1]
        if ch in ('u', 'i', 'o', 'j', 'l', 'm', ',', '.'):
            self.last_motion_key_time = time.time()
            self.auto_stop = False
        if ch == 'i':
            self.linear_x = self.speed
            self.angular_z = 0.0
        elif ch == ',':
            self.linear_x = -self.speed
            self.angular_z = 0.0
        elif ch == 'j':
            self.linear_x = 0.0
            self.angular_z = self.turn
        elif ch == 'l':
            self.linear_x = 0.0
            self.angular_z = -self.turn
        elif ch == 'u':
            self.linear_x = self.speed
            self.angular_z = self.turn
        elif ch == 'o':
            self.linear_x = self.speed
            self.angular_z = -self.turn
        elif ch == 'm':
            self.linear_x = -self.speed
            self.angular_z = -self.turn
        elif ch == '.':
            self.linear_x = -self.speed
            self.angular_z = self.turn
        elif ch in ('k', ' '):
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.auto_stop = False
        elif ch == 'w':
            self.speed = min(0.80, self.speed * 1.1)
        elif ch == 'x':
            self.speed = max(0.03, self.speed * 0.9)
        elif ch == 'e':
            self.turn = min(2.00, self.turn * 1.1)
        elif ch == 'c':
            self.turn = max(0.10, self.turn * 0.9)

    def publish(self):
        msg = Twist()
        msg.linear.x = float(self.linear_x)
        msg.angular.z = float(self.angular_z)
        self.pub.publish(msg)
        self.last_publish = time.time()
        self.msg_count += 1

    def bar(self, value, max_value, width=18):
        ratio = min(1.0, abs(value) / max_value) if max_value else 0.0
        filled = int(round(ratio * width))
        return '[' + ('#' * filled).ljust(width, '-') + ']'

    def draw(self):
        right, left = self.wheel_cmds()
        sys.stdout.write('\033[2J\033[H')
        print('MASTER WHEEL KEYBOARD CONTROL')
        print('=' * 58)
        suffix = ' (release detected)' if self.auto_stop else ''
        print(f'STATE     : {(self.state() + suffix):<33} LAST KEY: {self.last_key:<10} PUB: {self.msg_count}')
        print(f'CMD_VEL   : linear.x={self.linear_x:+.3f} m/s      angular.z={self.angular_z:+.3f} rad/s')
        print(f'RIGHT CMD : {right:+4d} {self.bar(right, self.max_speed_cmd)}')
        print(f'LEFT  CMD : {left:+4d} {self.bar(left, self.max_speed_cmd)}')
        print(f'SETTINGS  : speed={self.speed:.2f} m/s        turn={self.turn:.2f} rad/s')
        print('=' * 58)
        print('MOVE      : u/i/o   j/k/l   m/,/.')
        print('STOP      : k or SPACE')
        print('TUNE      : w/x linear speed up/down, e/c turn speed up/down')
        print('QUIT      : Ctrl-C sends STOP')
        print('')
        print(f'Mode: hold-to-drive. Hold movement key; release auto-stops after {self.hold_timeout:.2f}s.')
        print('Tuned for SSH key-repeat delay to avoid choppy stop/start.')
        sys.stdout.flush()

    def stop(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        for _ in range(4):
            self.publish()
            time.sleep(0.05)


def main():
    old_settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = PrettyTeleop()
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.draw()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                ch = sys.stdin.read(1)
                if ch == '\x03':
                    break
                node.handle_key(ch)
                node.publish()
                node.draw()
            else:
                if time.time() - node.last_motion_key_time > node.hold_timeout:
                    if abs(node.linear_x) > 0.001 or abs(node.angular_z) > 0.001:
                        node.linear_x = 0.0
                        node.angular_z = 0.0
                        node.last_key = 'AUTO_STOP'
                        node.auto_stop = True
                        node.publish()
                        node.draw()
                if time.time() - node.last_publish > 0.2:
                    node.publish()
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nSTOP command sent. Bye.')


if __name__ == '__main__':
    main()