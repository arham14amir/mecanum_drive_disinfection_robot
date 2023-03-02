import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinates, Coordinate
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PolygonStamped, Point32
import cv2
import numpy as np
import yaml
import os
import math

class RoomCoverer(Node):
    def __init__(self):
        super().__init__('room_coverer')
        
        # --- CONFIGURATION ---
        self.map_yaml_path = '/root/mecanum_drive_2/maps/my_map.yaml' 
        self.INVERT_Y_AXIS = False 
        
        # 1. CLIENT FOR CALCULATION (Coverage Server)
        self.coverage_client = ActionClient(self, ComputeCoveragePath, 'compute_coverage_path')
        
        # 2. CLIENT FOR DRIVING (Nav2 Controller)
        self.nav_client = ActionClient(self, FollowPath, 'follow_path')
        
        # Visualization
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.poly_pub = self.create_publisher(PolygonStamped, '/detected_room_polygon', latching_qos)
        self.path_pub = self.create_publisher(Path, '/coverage_path_preview', latching_qos)

    def pixel_to_world(self, u, v, origin, res, height):
        x = u * res + origin[0]
        if self.INVERT_Y_AXIS:
            y = v * res + origin[1]
        else:
            y = (height - v) * res + origin[1]
        return x, y

    def get_room_polygon(self):
        self.get_logger().info(f"Reading map: {self.map_yaml_path}")
        if not os.path.exists(self.map_yaml_path): return None

        with open(self.map_yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)
        
        res = map_data['resolution']
        origin = map_data['origin']
        img_path = os.path.join(os.path.dirname(self.map_yaml_path), map_data['image'])
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        
        # 1. Simplify Erosion (Less aggression reduces weird shapes)
        kernel = np.ones((3,3), np.uint8)
        img = cv2.erode(img, kernel, iterations=1) # Reduced from 2 to 1
        
        height, width = img.shape
        _, thresh = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None

        largest_contour = max(contours, key=cv2.contourArea)
        
        # --- CRITICAL FIX: AGGRESSIVE SMOOTHING ---
        # 0.03 forces the code to ignore small jagged bumps.
        # This turns a 28-corner jagged room into a clean 4-6 corner box.
        epsilon = 0.03 * cv2.arcLength(largest_contour, True) 
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)

        self.get_logger().info(f"Polygon simplified to {len(approx)} vertices.")

        coords_msg = Coordinates()
        viz_poly = PolygonStamped()
        viz_poly.header.frame_id = "map"
        viz_poly.header.stamp = self.get_clock().now().to_msg()
        
        temp_coords = []
        for point in approx:
            c = Coordinate()
            u, v = point[0][0], point[0][1]
            world_x, world_y = self.pixel_to_world(u, v, origin, res, height)
            c.axis1 = float(world_x)
            c.axis2 = float(world_y)
            temp_coords.append(c)

        # Enforce CCW
        area = 0.0
        for i in range(len(temp_coords)):
            j = (i + 1) % len(temp_coords)
            area += temp_coords[i].axis1 * temp_coords[j].axis2
            area -= temp_coords[j].axis1 * temp_coords[i].axis2
        if area < 0:
            temp_coords.reverse()

        coords_msg.coordinates = temp_coords
        coords_msg.coordinates.append(coords_msg.coordinates[0])

        for c in coords_msg.coordinates:
            p = Point32()
            p.x, p.y = c.axis1, c.axis2
            viz_poly.polygon.points.append(p)

        self.poly_pub.publish(viz_poly)
        return [coords_msg]

    def execute_coverage(self):
        # 1. Get Polygon
        polygons = self.get_room_polygon()
        if not polygons:
            self.get_logger().error("Could not find room in map!")
            return

        # 2. Prepare Calculation Goal
        goal_msg = ComputeCoveragePath.Goal()
        goal_msg.generate_headland = False
        goal_msg.generate_route = True
        goal_msg.generate_path = True
        goal_msg.polygons = polygons
        goal_msg.frame_id = "map"
        
        # Use DUBIN mode, but the Simplified Polygon prevents the crash.
        goal_msg.path_mode.mode = "DUBIN" 
        
        # 3. Send to Coverage Server
        self.get_logger().info("Requesting Path from Coverage Server...")
        self.coverage_client.wait_for_server()
        
        future = self.coverage_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Coverage Goal Rejected!")
            return
            
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        
        raw_path = result.nav_path
        self.get_logger().info(f"Received Path with {len(raw_path.poses)} points.")

        if len(raw_path.poses) == 0:
             self.get_logger().error("Path is empty! Check YAML 'default_path_continuity_type'.")
             return

        # 4. Publish for RViz
        self.path_pub.publish(raw_path)

        # 5. Send to Nav2 Controller
        follow_goal = FollowPath.Goal()
        follow_goal.path = raw_path
        follow_goal.controller_id = "FollowPath"

        self.get_logger().info("Sending Path to Nav2 Controller...")
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(follow_goal)
        self.get_logger().info("Robot should be moving now!")

def main():
    rclpy.init()
    node = RoomCoverer()
    node.execute_coverage()
    rclpy.spin(node)

if __name__ == '__main__':
    main()