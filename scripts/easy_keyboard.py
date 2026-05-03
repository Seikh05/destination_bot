#!/usr/bin/env python3
"""
Easy WASD keyboard controller.
Just like a video game — no weird key combos!

W = Forward
S = Backward  
A = Turn Left
D = Turn Right
Q = Spin Left
E = Spin Right
SPACE = STOP
X = Quit
"""
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── Key bindings ───────────────────────────────────────────────────────
KEYS = {
    'w': ( 1,  0),    # forward
    's': (-1,  0),    # backward
    'a': ( 0,  1),    # turn left
    'd': ( 0, -1),    # turn right
    'q': ( 0,  1),    # spin left  (same as a but no forward)
    'e': ( 0, -1),    # spin right
    ' ': ( 0,  0),    # full stop
}

BANNER = """
╔══════════════════════════════════════╗
║       EASY ROBOT CONTROLLER          ║
╠══════════════════════════════════════╣
║   W  = Forward                       ║
║   S  = Backward                      ║
║   A  = Turn Left                     ║
║   D  = Turn Right                    ║
║   Q  = Spin Left (in place)          ║
║   E  = Spin Right (in place)         ║
║ SPC  = STOP                          ║
║   +  = Speed Up                      ║
║   -  = Slow Down                     ║
║   X  = Quit                          ║
╚══════════════════════════════════════╝
Hold key = keep moving
Release  = robot stops automatically
"""


class EasyKeyboard(Node):
    def __init__(self):
        super().__init__('easy_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed  = 0.20   # m/s
        self.angular_speed = 0.60   # rad/s

        print(BANNER)
        print(f'  Speed: {self.linear_speed:.2f} m/s  '
              f'Turn: {self.angular_speed:.2f} rad/s\n')

    def get_key(self):
        """Read a single keypress without needing Enter."""
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key.lower()

    def send(self, lin, ang):
        t = Twist()
        t.linear.x  = lin * self.linear_speed
        t.angular.z = ang * self.angular_speed
        self.pub.publish(t)

    def stop(self):
        self.send(0, 0)

    def run(self):
        print('  Ready! Press keys to drive...\n')
        while rclpy.ok():
            key = self.get_key()

            # Quit
            if key == 'x':
                print('\n  Stopping robot and quitting...')
                self.stop()
                break

            # Speed up / down
            elif key == '+' or key == '=':
                self.linear_speed  = min(0.50, self.linear_speed  + 0.05)
                self.angular_speed = min(1.50, self.angular_speed + 0.10)
                print(f'  Speed: {self.linear_speed:.2f} m/s  '
                      f'Turn: {self.angular_speed:.2f} rad/s')
                continue

            elif key == '-' or key == '_':
                self.linear_speed  = max(0.05, self.linear_speed  - 0.05)
                self.angular_speed = max(0.20, self.angular_speed - 0.10)
                print(f'  Speed: {self.linear_speed:.2f} m/s  '
                      f'Turn: {self.angular_speed:.2f} rad/s')
                continue

            # Movement keys
            elif key in KEYS:
                lin, ang = KEYS[key]

                # Q and E = pure rotation (no forward component)
                if key in ('q', 'e'):
                    lin = 0.0

                self.send(lin, ang)

                # Show what's happening
                actions = {
                    'w': 'Forward  ↑',
                    's': 'Backward ↓',
                    'a': 'Turn Left ←',
                    'd': 'Turn Right →',
                    'q': 'Spinning Left ↺',
                    'e': 'Spinning Right ↻',
                    ' ': 'STOPPED ■',
                }
                print(f'  {actions.get(key, "")}', end='\r')

            # Any other key = stop
            else:
                self.stop()
                print(f'  STOPPED ■          ', end='\r')


def main():
    rclpy.init()
    node = EasyKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print('\n  Done!')


if __name__ == '__main__':
    main()