#!/usr/bin/env python3
"""
Adaptive autonomous explorer.
Auto-detects 180° or 360° lidar and adjusts accordingly.
Uses wall-corner escape maneuver.
"""
import rclpy
import random
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_timer(0.05, self.control_loop)

        # ── Distances ──────────────────────────────────────────────────
        self.front       = float('inf')
        self.front_left  = float('inf')
        self.front_right = float('inf')
        self.left        = float('inf')
        self.right       = float('inf')
        self.rear        = float('inf')

        # ── Lidar properties (set on first scan) ───────────────────────
        self.angle_min   = None
        self.angle_max   = None
        self.is_360      = False
        self.scan_ready  = False

        # ── State machine ──────────────────────────────────────────────
        self.state     = 'forward'
        self.state_end = 0.0
        self.turn_dir  = 1.0

        # ── Stuck detection ────────────────────────────────────────────
        self.stuck_timer    = time.time()
        self.stuck_x        = 0.0
        self.stuck_y        = 0.0
        self.escape_mode    = False
        self.escape_end     = 0.0

        # ── Speed params ───────────────────────────────────────────────
        self.LIN = 0.15     # m/s
        self.ANG = 0.80     # rad/s
        self.D1  = 0.40     # DANGER distance
        self.D2  = 0.65     # CAUTION distance

        self.get_logger().info('Adaptive Explorer starting...')

    # ──────────────────────────────────────────────────────────────────
    def idx_at_angle(self, ranges, target_rad, half_width_rad):
        """
        Get min range in a cone at target_rad from scan front.
        Works for both 180° and 360° lidars.
        target_rad: angle in radians from forward direction
                    0=front, pi/2=left, pi=back, -pi/2=right
        """
        if self.angle_min is None:
            return float('inf')

        angle_range = self.angle_max - self.angle_min
        n = len(ranges)

        vals = []
        steps = int(math.degrees(half_width_rad)) + 1
        for offset_deg in range(-steps, steps + 1):
            offset_rad = math.radians(offset_deg)
            angle = target_rad + offset_rad

            # Check if this angle is within lidar range
            if angle < self.angle_min or angle > self.angle_max:
                continue

            # Map angle to index
            idx = int((angle - self.angle_min) / angle_range * (n - 1))
            idx = max(0, min(n - 1, idx))

            r = ranges[idx]
            if math.isfinite(r) and r > 0.05:
                vals.append(r)

        return min(vals) if vals else float('inf')

    def scan_callback(self, msg: LaserScan):
        # First scan — detect lidar type
        if self.angle_min is None:
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max
            span = math.degrees(msg.angle_max - msg.angle_min)
            self.is_360 = span >= 340

            self.get_logger().info(
                f'Lidar detected: {span:.1f}° scan '
                f'({"360°" if self.is_360 else "180° — LIMITED!"})'
            )
            if not self.is_360:
                self.get_logger().warn(
                    'Only 180° lidar! Robot is BLIND behind it. '
                    'Using conservative thresholds.')
                # Be more cautious with limited lidar
                self.D1 = 0.50
                self.D2 = 0.80

        r = list(msg.ranges)

        # Standard angles (radians from forward)
        self.front       = self.idx_at_angle(r,  0,           math.radians(25))
        self.front_left  = self.idx_at_angle(r,  math.pi/4,  math.radians(15))
        self.front_right = self.idx_at_angle(r, -math.pi/4,  math.radians(15))
        self.left        = self.idx_at_angle(r,  math.pi/2,  math.radians(25))
        self.right       = self.idx_at_angle(r, -math.pi/2,  math.radians(25))

        # Rear only available on 360° lidar
        if self.is_360:
            self.rear = self.idx_at_angle(r, math.pi, math.radians(25))
        else:
            self.rear = float('inf')   # assume clear if can't see

        self.scan_ready = True

    # ──────────────────────────────────────────────────────────────────
    def set_state(self, new_state, duration):
        if new_state != self.state:
            self.get_logger().info(
                f'  [{self.state}] → [{new_state}]  '
                f'F={self.front:.2f} '
                f'FL={self.front_left:.2f} '
                f'FR={self.front_right:.2f} '
                f'L={self.left:.2f} '
                f'R={self.right:.2f}'
            )
        self.state     = new_state
        self.state_end = time.time() + duration

    def state_done(self):
        return time.time() >= self.state_end

    def best_turn(self):
        if self.left >= self.right:
            return 1.0    # more space on left → turn left
        return -1.0       # more space on right → turn right

    # ──────────────────────────────────────────────────────────────────
    def control_loop(self):
        if not self.scan_ready:
            return

        t = Twist()

        # ── ESCAPE mode: robot was stuck, doing full spin escape ───────
        if self.escape_mode:
            if time.time() < self.escape_end:
                t.linear.x  = -self.LIN   # back up while spinning
                t.angular.z =  self.ANG * self.turn_dir
                self.cmd_pub.publish(t)
                return
            else:
                self.escape_mode = False
                self.set_state('forward', 999)
                self.get_logger().info('Escape complete — resuming')

        # ── BACKUP ────────────────────────────────────────────────────
        if self.state == 'backup':
            if self.is_360 and self.rear < 0.25:
                self.get_logger().warn('Wall behind! Force turn.')
                self.turn_dir = self.best_turn()
                self.set_state('turn', random.uniform(2.5, 4.0))
            elif not self.state_done():
                t.linear.x  = -self.LIN
                t.angular.z = 0.0
            else:
                self.turn_dir = self.best_turn()
                self.set_state('turn', random.uniform(2.5, 4.5))

        # ── TURN ──────────────────────────────────────────────────────
        elif self.state == 'turn':
            if not self.state_done():
                t.linear.x  = 0.0
                t.angular.z = self.turn_dir * self.ANG
            else:
                if self.front < self.D2:
                    self.get_logger().info(
                        f'Still blocked {self.front:.2f}m — more turning')
                    self.set_state('turn', random.uniform(2.0, 3.5))
                else:
                    self.set_state('forward', 999)

        # ── FORWARD ───────────────────────────────────────────────────
        else:
            if self.front < self.D1:
                self.get_logger().warn(f'DANGER {self.front:.2f}m!')
                self.set_state('backup', random.uniform(2.0, 3.0))
                t.linear.x  = -self.LIN
                t.angular.z = 0.0

            elif self.front < self.D2:
                self.turn_dir = self.best_turn()
                self.set_state('turn', random.uniform(2.5, 4.0))
                t.linear.x  = 0.0
                t.angular.z = self.turn_dir * self.ANG

            elif self.front_left < self.D2 and self.front_right < self.D2:
                # Narrow corridor — go straight slowly
                t.linear.x  = self.LIN * 0.5
                t.angular.z = 0.0

            elif self.front_left < self.D2:
                t.linear.x  = self.LIN * 0.6
                t.angular.z = -self.ANG * 0.5   # steer right

            elif self.front_right < self.D2:
                t.linear.x  = self.LIN * 0.6
                t.angular.z =  self.ANG * 0.5   # steer left

            elif self.left < 0.30:
                t.linear.x  = self.LIN
                t.angular.z = -self.ANG * 0.3

            elif self.right < 0.30:
                t.linear.x  = self.LIN
                t.angular.z =  self.ANG * 0.3

            else:
                t.linear.x  = self.LIN
                t.angular.z = random.uniform(-0.06, 0.06)

        self.cmd_pub.publish(t)

    def stop(self):
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Stopped.')


def main():
    rclpy.init()
    node = AutonomousExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()