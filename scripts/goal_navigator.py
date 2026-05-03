#!/usr/bin/env python3
"""
Simple goal navigator.
After SLAM map is saved, run this node and type target
coordinates. The robot drives there avoiding obstacles
using the lidar.

Usage:
  ros2 run my_robot goal_navigator.py
  Then type: 2.0 1.5   (x y coordinates)
"""
import rclpy
import math
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry,   '/odom', self.odom_cb,  10)
        self.create_subscription(LaserScan,  '/scan', self.scan_cb,  10)
        self.create_timer(0.1, self.control_loop)

        # Robot pose (updated from odom)
        self.x     = 0.0
        self.y     = 0.0
        self.yaw   = 0.0

        # Goal
        self.goal_x     = None
        self.goal_y     = None
        self.goal_active = False

        # Lidar
        self.front_dist = float('inf')

        # Tuning
        self.linear_speed    = 0.20
        self.angular_speed   = 0.60
        self.goal_tolerance  = 0.25   # metres — close enough to goal
        self.obstacle_dist   = 0.50   # metres — stop if blocked

        self.get_logger().info('Goal Navigator ready!')
        self.get_logger().info('Starting input thread — type: x y (e.g. 2.0 1.5)')

        # Input thread so we can type goals without blocking ROS spin
        t = threading.Thread(target=self.input_loop, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Convert quaternion → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def scan_cb(self, msg):
        ranges = msg.ranges
        n = len(ranges)
        cone = int(25 * n / 360)
        front_idx = list(range(0, cone)) + list(range(n - cone, n))
        vals = [ranges[i] for i in front_idx
                if not math.isnan(ranges[i]) and not math.isinf(ranges[i])]
        self.front_dist = min(vals) if vals else float('inf')

    # ------------------------------------------------------------------
    def input_loop(self):
        """Runs in separate thread — waits for user to type goal."""
        while True:
            try:
                raw = input('\n📍 Enter goal (x y) or q to quit: ').strip()
                if raw.lower() == 'q':
                    self.goal_active = False
                    self.get_logger().info('Goal cancelled.')
                    continue
                parts = raw.split()
                if len(parts) != 2:
                    print('   ⚠️  Enter two numbers: x y')
                    continue
                gx, gy = float(parts[0]), float(parts[1])
                self.goal_x     = gx
                self.goal_y     = gy
                self.goal_active = True
                self.get_logger().info(f'New goal set: ({gx:.2f}, {gy:.2f})')
            except (ValueError, EOFError):
                print('   ⚠️  Invalid input. Try: 2.0 1.5')

    # ------------------------------------------------------------------
    def control_loop(self):
        twist = Twist()

        if not self.goal_active or self.goal_x is None:
            self.cmd_pub.publish(twist)   # stay still
            return

        # Distance to goal
        dx   = self.goal_x - self.x
        dy   = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        # ── Reached goal ───────────────────────────────────────────────
        if dist < self.goal_tolerance:
            self.goal_active = False
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f'✅ Goal reached! ({self.goal_x:.2f}, {self.goal_y:.2f})')
            print(f'\n✅ Reached goal! Enter next goal (x y): ', end='', flush=True)
            return

        # ── Obstacle blocking path ─────────────────────────────────────
        if self.front_dist < self.obstacle_dist:
            # Stop and rotate until clear
            twist.linear.x  = 0.0
            twist.angular.z = self.angular_speed
            self.cmd_pub.publish(twist)
            return

        # ── Angle to goal ──────────────────────────────────────────────
        target_angle = math.atan2(dy, dx)
        angle_error  = target_angle - self.yaw

        # Normalize to [-pi, pi]
        while angle_error >  math.pi: angle_error -= 2 * math.pi
        while angle_error < -math.pi: angle_error += 2 * math.pi

        # ── Rotate to face goal first ──────────────────────────────────
        if abs(angle_error) > 0.3:   # ~17 degrees
            twist.linear.x  = 0.0
            twist.angular.z = self.angular_speed * (1.0 if angle_error > 0 else -1.0)

        # ── Drive toward goal ──────────────────────────────────────────
        else:
            # Slow down as we get close
            speed = min(self.linear_speed, dist * 0.5)
            twist.linear.x  = max(0.05, speed)
            twist.angular.z = angle_error * 1.5   # proportional correction

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()