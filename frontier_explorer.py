#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float64MultiArray, Bool
from math import hypot
from collections import deque
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf2_ros
import rclpy.time
from geometry_msgs.msg import TransformStamped

UNKNOWN = -1


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_pub = self.create_publisher(Float64MultiArray, '/global_waypoints', 10)
        self.stop_threshold = 0.8  
        self.stop_pub = self.create_publisher(Bool, '/stop_robot', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.map = None
        self.map_np = None
        self.pose = None          
        self.last_published_goal = None
        self.kickstarted = False  
        self.min_frontier_size = 5
        self.info_radius = 2.0   
        self.replan_period = 5.0 
        self.timer = self.create_timer(self.replan_period, self.explore)
    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
        self.map_np = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)





    def odom_callback(self, msg: Odometry):
        self.pose = msg.pose.pose

    
    def get_pose_in_map(self):
        if self.map is None:
            return None
        map_frame = self.map.header.frame_id if hasattr(self.map, 'header') else 'map'
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                map_frame, 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.5)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            class _P: pass
            p = _P()
            p.position = type('pt', (), {'x': x, 'y': y, 'z': 0.0})
            return p
        except Exception:
            return None





    def explore(self):
        if self.map_np is None:
            return

        map_pose = self.get_pose_in_map() or self.pose
        if map_pose is None:
            return



        free = int(np.count_nonzero(self.map_np == 0))
        unknown = int(np.count_nonzero(self.map_np == UNKNOWN))
        occ = int(np.count_nonzero(self.map_np == 100))
        self.get_logger().info(f"Map stats: free={free}, unknown={unknown}, occ={occ}")
        total_cells = self.map_np.size
        known_cells = np.count_nonzero((self.map_np == 0) | (self.map_np == 100))
        explored_fraction = known_cells / total_cells






        self.get_logger().info(f"Explored fraction: {explored_fraction:.2f}")

        if explored_fraction >= self.stop_threshold:
            self.get_logger().info("Exploration complete! Stopping robot.")
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
            self.timer.cancel()
            return
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        sx = int((map_pose.position.x - ox) / res)
        sy = int((map_pose.position.y - oy) / res)

        sx = max(0, min(self.map_np.shape[1]-1, sx))
        sy = max(0, min(self.map_np.shape[0]-1, sy))

        self.get_logger().info(f"Robot pose: ({map_pose.position.x:.2f}, {map_pose.position.y:.2f}) -> grid ({sx},{sy})")


        if np.count_nonzero(self.map_np == 0) < 10:
            self.get_logger().info("Waiting for map to initialize with free space...")
            return
        frontiers = self.find_frontiers(self.map_np)
        clusters = self.cluster_frontiers(frontiers)

        if not clusters:
            if not self.kickstarted:
                self.get_logger().warn("No frontiers yet. Kickstarting exploration...")
                x = map_pose.position.x + 0.5
                y = map_pose.position.y
                msg = Float64MultiArray()
                msg.data = [x, y]
                self.goal_pub.publish(msg)
                self.get_logger().info(f"Kickstart waypoint published: ({x:.2f}, {y:.2f})")
                self.kickstarted = True
            return

        candidates = []
        for cluster in clusters:
            candidate = self.cluster_centroid(cluster)
            if candidate is None:
                continue
            path_cost = self.a_star_cost(candidate)
            if path_cost is None:
                continue
            info = self.info_gain(candidate, self.info_radius)
            score = info - 1.5 * path_cost
            candidates.append((score, candidate))

        if not candidates:
            self.get_logger().warn("No reachable frontier candidates found.")
            return

        all_goals = [c[1] for c in candidates]




        
        path_msg = Path()
        path_msg.header.frame_id = self.map.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y

        for goal in all_goals:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = goal[0]
            pose.pose.position.y = goal[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        flat_array = Float64MultiArray()
        for goal in all_goals:
            flat_array.data.extend([goal[0], goal[1]])

        current_goals = list(flat_array.data) 

        if self.last_published_goal is None or \
            len(current_goals) != len(self.last_published_goal) or \
            not np.allclose(current_goals, self.last_published_goal, atol=0.1):
                
                self.goal_pub.publish(flat_array)
                self.get_logger().info(f"Published {len(all_goals)} frontier waypoints.")
                self.last_published_goal = current_goals

    def find_frontiers(self, grid):
        H, W = grid.shape
        frontiers = []
        for y in range(1, H-1):
            for x in range(1, W-1):
                if grid[y, x] == 0:
                    if (grid[y+1, x] == UNKNOWN or grid[y-1, x] == UNKNOWN or
                        grid[y, x+1] == UNKNOWN or grid[y, x-1] == UNKNOWN):
                        frontiers.append((x, y))
        return frontiers

    def cluster_frontiers(self, frontiers):
        clusters = []
        seen = set()
        for f in frontiers:
            if f in seen:
                continue
            cluster = []
            q = deque([f])
            seen.add(f)
            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for nx, ny in [(cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)]:
                    if (nx, ny) in frontiers and (nx, ny) not in seen:
                        seen.add((nx, ny))
                        q.append((nx, ny))
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)
        return clusters

    def cluster_centroid(self, cluster):
        xs = [c[0] for c in cluster]
        ys = [c[1] for c in cluster]
        cx = int(sum(xs) / len(xs))
        cy = int(sum(ys) / len(ys))
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        wx = ox + cx * res
        wy = oy + cy * res
        theta = 0.0
        if self.map_np[cy, cx] != 0:
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    nx, ny = cx + dx, cy + dy
                    if (0 <= nx < self.map_np.shape[1] and
                        0 <= ny < self.map_np.shape[0] and
                        self.map_np[ny, nx] == 0):
                        wx = ox + nx * res
                        wy = oy + ny * res
                        return (wx, wy, theta)
        return (wx, wy, theta)

    def a_star_cost(self, goal):
        if self.map_np is None:
            return None
        grid = self.map_np
        H, W = grid.shape
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        start_p = self.get_pose_in_map() or self.pose
        if start_p is None:
            return None
        sx = int((self.pose.position.x - ox) / res + 0.5)
        sy = int((self.pose.position.y - oy) / res + 0.5)
        gx = int((goal[0] - ox) / res + 0.5)
        gy = int((goal[1] - oy) / res + 0.5)
        sx = max(0, min(W - 1, sx))
        sy = max(0, min(H - 1, sy))
        gx = max(0, min(W - 1, gx))
        gy = max(0, min(H - 1, gy))




        if not (0 <= sx < W and 0 <= sy < H and 0 <= gx < W and 0 <= gy < H):
            return None
        if grid[gy, gx] == 100:
            return None

        open_set = [(0, (sx, sy))]
        costs = {(sx, sy): 0}
        visited = set()
        while open_set:
            cost, (x, y) = open_set.pop(0)
            if (x, y) == (gx, gy):
                return costs[(x, y)] * res
            if (x, y) in visited:
                continue
            visited.add((x, y))
            for nx, ny in [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]:
                if 0 <= nx < W and 0 <= ny < H and grid[ny, nx] == 0:
                    new_cost = costs[(x, y)] + 1
                    if (nx, ny) not in costs or new_cost < costs[(nx, ny)]:
                        costs[(nx, ny)] = new_cost
                        priority = new_cost + hypot(nx-gx, ny-gy)
                        open_set.append((priority, (nx, ny)))
                        open_set.sort(key=lambda c: c[0])
        return None
    
    
    
    
    def info_gain(self, goal, radius):
        grid = self.map_np
        H, W = grid.shape
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        gx = int((goal[0] - ox) / res + 0.5)
        gy = int((goal[1] - oy) / res + 0.5)
        gx = max(0, min(W - 1, gx))
        gy = max(0, min(H - 1, gy))
        rad = int(radius / res)
        cnt = 0
        for dy in range(-rad, rad+1):
            for dx in range(-rad, rad+1):
                x = gx + dx
                y = gy + dy
                if 0 <= x < W and 0 <= y < H and grid[y, x] == UNKNOWN:
                    cnt += 1
        return cnt


def main():
    rclpy.init()
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
