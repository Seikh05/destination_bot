#!/usr/bin/env python3
import heapq, math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.map_data = None
        self.map_info = None

        # Subscribe to saved map
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        # Publish the found path (visible in RViz)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to start and goal points
        # Send via: ros2 topic pub /start_point ...
        self.create_subscription(PoseStamped, '/start_point', self.start_cb, 10)
        self.create_subscription(PoseStamped, '/goal_point',  self.goal_cb,  10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(0.1, self.follow_path_cb)

        self.start = None
        self.goal  = None

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.path_world = []
        self.waypoint_index = 0
        self.path_active = False

        self.waypoint_tolerance = 0.12
        self.goal_tolerance = 0.20
        self.max_linear = 0.20
        self.max_angular = 0.90
        self.auto_project_points = True

        self.get_logger().info('A* Planner node started.')

    def map_cb(self, msg):
        self.map_info = msg.info
        w = msg.info.width
        h = msg.info.height
        self.map_data = [
            [msg.data[r * w + c] > 50
             for c in range(w)]
            for r in range(h)
        ]
        min_x = msg.info.origin.position.x
        min_y = msg.info.origin.position.y
        max_x = min_x + w * msg.info.resolution
        max_y = min_y + h * msg.info.resolution
        self.get_logger().info(
            f'Map loaded: {w} x {h}, bounds x:[{min_x:.2f},{max_x:.2f}] y:[{min_y:.2f},{max_y:.2f}]'
        )

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def start_cb(self, msg):
        self.start = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Start set: {self.start}')
        self.try_plan()

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Goal set: {self.goal}')
        self.try_plan()

    def try_plan(self):
        if self.start and self.goal and self.map_data:
            sg_raw = self.world_to_grid(*self.start)
            gg_raw = self.world_to_grid(*self.goal)

            sg = self.project_to_valid_cell(sg_raw, 'start') if self.auto_project_points else sg_raw
            gg = self.project_to_valid_cell(gg_raw, 'goal') if self.auto_project_points else gg_raw

            if sg is None or gg is None:
                self.path_active = False
                self.cmd_pub.publish(Twist())
                self.get_logger().warn('Cannot create valid start/goal cells from requested points.')
                return

            path = self.astar(sg, gg)
            if path:
                self.publish_path(path)
                self.start_follower(path)
                self.get_logger().info(f'Path found: {len(path)} steps')
            else:
                self.path_active = False
                self.cmd_pub.publish(Twist())
                self.get_logger().warn('No path found!')

    def world_to_grid(self, wx, wy):
        res = self.map_info.resolution
        ox  = self.map_info.origin.position.x
        oy  = self.map_info.origin.position.y
        return int((wy - oy) / res), int((wx - ox) / res)

    def grid_to_world(self, row, col):
        res = self.map_info.resolution
        ox  = self.map_info.origin.position.x
        oy  = self.map_info.origin.position.y
        return ox + col * res + res / 2.0, oy + row * res + res / 2.0

    def astar(self, start, goal):
        rows = len(self.map_data)
        cols = len(self.map_data[0])
        h = lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1])

        open_heap = [(0.0, start)]
        came_from = {start: None}
        g = {start: 0.0}

        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur == goal:
                path = []
                while cur:
                    path.append(cur)
                    cur = came_from[cur]
                return path[::-1]

            for dr in (-1,0,1):
                for dc in (-1,0,1):
                    if dr == dc == 0: continue
                    nr, nc = cur[0]+dr, cur[1]+dc
                    if 0 <= nr < rows and 0 <= nc < cols and not self.map_data[nr][nc]:
                        ng = g[cur] + math.hypot(dr, dc)
                        nb = (nr, nc)
                        if nb not in g or ng < g[nb]:
                            g[nb] = ng
                            heapq.heappush(open_heap, (ng + h(nb, goal), nb))
                            came_from[nb] = cur
        return None

    def is_in_bounds(self, row, col):
        rows = len(self.map_data)
        cols = len(self.map_data[0])
        return 0 <= row < rows and 0 <= col < cols

    def project_to_valid_cell(self, cell, label):
        rows = len(self.map_data)
        cols = len(self.map_data[0])
        row, col = cell

        row_clamped = min(max(row, 0), rows - 1)
        col_clamped = min(max(col, 0), cols - 1)
        adjusted = (row_clamped != row) or (col_clamped != col)

        if adjusted:
            self.get_logger().warn(
                f'{label.capitalize()} cell {cell} is out of bounds; clamped to {(row_clamped, col_clamped)}.'
            )

        if not self.map_data[row_clamped][col_clamped]:
            return (row_clamped, col_clamped)

        max_radius = max(rows, cols)
        for rad in range(1, max_radius):
            r0 = max(0, row_clamped - rad)
            r1 = min(rows - 1, row_clamped + rad)
            c0 = max(0, col_clamped - rad)
            c1 = min(cols - 1, col_clamped + rad)

            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    if (r in (r0, r1) or c in (c0, c1)) and not self.map_data[r][c]:
                        self.get_logger().warn(
                            f'{label.capitalize()} cell {(row_clamped, col_clamped)} occupied; projected to nearest free {(r, c)}.'
                        )
                        return (r, c)

        self.get_logger().warn(f'No free cell available for {label} near {cell}.')
        return None

    def start_follower(self, grid_path):
        self.path_world = [self.grid_to_world(r, c) for r, c in grid_path]
        self.waypoint_index = 0
        self.path_active = len(self.path_world) > 0

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def follow_path_cb(self):
        if not self.path_active or not self.path_world:
            return

        if self.waypoint_index >= len(self.path_world):
            self.path_active = False
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Path execution complete.')
            return

        tx, ty = self.path_world[self.waypoint_index]
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        dist = math.hypot(dx, dy)

        final_wp = self.waypoint_index == len(self.path_world) - 1
        tol = self.goal_tolerance if final_wp else self.waypoint_tolerance

        if dist < tol:
            self.waypoint_index += 1
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - self.robot_yaw)

        cmd = Twist()
        if abs(yaw_err) > 0.35:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, 1.8 * yaw_err))
        else:
            cmd.linear.x = min(self.max_linear, max(0.05, 0.6 * dist))
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, 1.5 * yaw_err))

        self.cmd_pub.publish(cmd)

    def publish_path(self, grid_path):
        msg = Path()
        msg.header = Header(frame_id='map')
        msg.header.stamp = self.get_clock().now().to_msg()
        for row, col in grid_path:
            wx, wy = self.grid_to_world(row, col)
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = wx
            p.pose.position.y = wy
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(AStarPlanner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()