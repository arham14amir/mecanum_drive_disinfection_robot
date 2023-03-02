import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PolygonStamped, Point32
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinates, Coordinate
from nav2_msgs.action import NavigateToPose
import cv2
import numpy as np
import math
import os

# ============================================================
#  SET THIS TO False WHEN YOU'RE READY TO MOVE REAL HARDWARE
# ============================================================
RVIZ_PREVIEW_ONLY = False

# ============================================================
#  DEBUG: saves processed map images to /tmp/
# ============================================================
SAVE_DEBUG_IMAGES = True


class AutoCoverageExecute(Node):
    def __init__(self):
        super().__init__('auto_coverage_execute')

        self.coverage_client = ActionClient(self, ComputeCoveragePath, 'compute_coverage_path')

        self.path_pub       = self.create_publisher(Path,           '/map_coverage_path',  10)
        self.waypoint_pub   = self.create_publisher(PoseArray,      '/coverage_waypoints', 10)
        self.boundary_pub   = self.create_publisher(PolygonStamped, '/coverage_boundary',  10)

        self.final_path       = None
        self.final_waypoints  = None
        self.boundary_polygon = None

        self.timer = self.create_timer(1.0, self.timer_callback)

        # --- MAP SETTINGS ---
        self.map_path   = '/root/mecanum_drive_2/maps/department.pgm'
        self.resolution = 0.05
        self.origin_x   = -2.41
        self.origin_y   = -1.64

        # --- TUNING PARAMS ---
        self.closing_size = 7          
        self.simplify_epsilon = 2.0    
        self.obs_dilate_iter = 2       
        self.min_hole_fraction = 0.02  

        # --- HW EXECUTION STATE ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose_queue = []
        self.watchdog = None
        self.current_goal_handle = None

        # Retry Timer Variable
        self._retry_timer = None

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        if self.final_path:
            self.final_path.header.stamp = now
            self.path_pub.publish(self.final_path)
        if self.final_waypoints:
            self.final_waypoints.header.stamp = now
            self.waypoint_pub.publish(self.final_waypoints)
        if self.boundary_polygon:
            self.boundary_polygon.header.stamp = now
            self.boundary_pub.publish(self.boundary_polygon)

    def px_to_world(self, px, py, height_px):
        mx = px * self.resolution + self.origin_x
        my = (height_px - py) * self.resolution + self.origin_y
        return mx, my

    def contour_to_coords(self, contour, height_px):
        coords = []
        for pt in contour:
            px, py = int(pt[0][0]), int(pt[0][1])
            mx, my = self.px_to_world(px, py, height_px)
            coords.append(Coordinate(axis1=mx, axis2=my))
        if coords:
            coords.append(coords[0])  
        return coords

    def process_and_send(self):
        if not os.path.exists(self.map_path):
            self.get_logger().error(f"Map not found: {self.map_path}")
            return

        self.get_logger().info(f"Loading map for processing: {self.map_path}")
        img = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error("Failed to load map image.")
            return

        height_px, width_px = img.shape

        # STEP 1: OUTER BOUNDARY
        _, free_mask = cv2.threshold(img, 250, 255, cv2.THRESH_BINARY)
        close_kernel = np.ones((self.closing_size, self.closing_size), np.uint8)
        free_closed = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, close_kernel)

        outer_contours, _ = cv2.findContours(free_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not outer_contours:
            self.get_logger().error("No outer boundary found.")
            return

        outer_contour = sorted(outer_contours, key=cv2.contourArea, reverse=True)[0]
        outer_area_m2 = cv2.contourArea(outer_contour) * (self.resolution ** 2)
        approx_outer = cv2.approxPolyDP(outer_contour, self.simplify_epsilon, True)

        room_mask = np.zeros_like(img)
        cv2.drawContours(room_mask, [outer_contour], -1, 255, thickness=cv2.FILLED)

        # STEP 2: INTERNAL OBSTACLES
        _, obs_mask = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)
        obs_inside = cv2.bitwise_and(obs_mask, room_mask)
        
        kernel = np.ones((3,3), np.uint8)
        obs_dilated = cv2.dilate(obs_inside, kernel, iterations=self.obs_dilate_iter)
        obs_final_mask = cv2.bitwise_and(obs_dilated, room_mask)

        internal_holes = []
        obs_contours, _ = cv2.findContours(obs_final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in obs_contours:
            hole_area_m2 = cv2.contourArea(cnt) * (self.resolution ** 2)
            if (hole_area_m2 / outer_area_m2) < self.min_hole_fraction:
                continue 
            
            approx_hole = cv2.approxPolyDP(cnt, 1.0, True)
            internal_holes.append(approx_hole)

        if SAVE_DEBUG_IMAGES:
            dbg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(dbg, [approx_outer], -1, (0, 255, 0), 2) 
            for hole in internal_holes:
                cv2.drawContours(dbg, [hole], -1, (0, 0, 255), 2)    
            cv2.imwrite('/tmp/coverage_debug.png', dbg)
            self.get_logger().info("Debug overlay saved to /tmp/coverage_debug.png")

        # STEP 3: CONVERT & SEND
        outer_coords = self.contour_to_coords(approx_outer, height_px)
        outer_poly = Coordinates()
        outer_poly.coordinates = outer_coords

        hole_polys = []
        for h in internal_holes:
            p = Coordinates()
            p.coordinates = self.contour_to_coords(h, height_px)
            hole_polys.append(p)

        self.boundary_polygon = PolygonStamped()
        self.boundary_polygon.header.frame_id = 'map'
        self.boundary_polygon.header.stamp = self.get_clock().now().to_msg()
        for c in outer_coords:
            self.boundary_polygon.polygon.points.append(Point32(x=c.axis1, y=c.axis2, z=0.0))
        self.boundary_pub.publish(self.boundary_polygon)

        # Save goal message to class state so we can retry it if needed
        self._coverage_goal_msg = ComputeCoveragePath.Goal()
        self._coverage_goal_msg.polygons = [outer_poly] + hole_polys

        self.get_logger().info(f"Sending coverage goal with {len(hole_polys)} obstacles...")
        self.coverage_client.wait_for_server()
        
        # Trigger the first attempt
        self._send_coverage_goal()

    # --- NEW: Dedicated send function so we can loop it ---
    def _send_coverage_goal(self):
        self.coverage_client.send_goal_async(self._coverage_goal_msg).add_done_callback(self.coverage_response_callback)

    def coverage_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("Coverage goal REJECTED by server. Server might still be booting. Retrying in 2 seconds...")
            # Use ROS 2 timer for a safe asynchronous wait that won't lock up the system
            self._retry_timer = self.create_timer(2.0, self._retry_timer_callback)
            return
            
        self.get_logger().info("Coverage goal ACCEPTED! Robot is starting.")
        gh.get_result_async().add_done_callback(self.coverage_result_callback)

    # --- NEW: Timer callback to trigger the next attempt ---
    def _retry_timer_callback(self):
        self._retry_timer.cancel() # Stop the timer so it doesn't fire twice
        self.get_logger().info("Retrying to send coverage goal...")
        self._send_coverage_goal()

    def euler_to_quaternion(self, yaw):
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def extract_waypoints(self, path):
        if len(path.poses) < 3: return path.poses
        raw = [path.poses[0]]
        p0, p1 = path.poses[0].pose.position, path.poses[1].pose.position
        cur_angle = math.atan2(p1.y - p0.y, p1.x - p0.x)

        for i in range(1, len(path.poses) - 1):
            a, b = path.poses[i].pose.position, path.poses[i+1].pose.position
            if math.hypot(b.x - a.x, b.y - a.y) < 0.01: continue
            nxt_angle = math.atan2(b.y - a.y, b.x - a.x)
            if abs(math.atan2(math.sin(nxt_angle - cur_angle), math.cos(nxt_angle - cur_angle))) > 0.43:
                raw.append(path.poses[i])
                cur_angle = nxt_angle
        raw.append(path.poses[-1])

        fixed = []
        for i in range(len(raw) - 1):
            curr, nxt = raw[i].pose, raw[i+1].pose
            yaw = math.atan2(nxt.position.y - curr.position.y, nxt.position.x - curr.position.x)
            p = Pose()
            p.position = curr.position
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = self.euler_to_quaternion(yaw)
            s = raw[i]
            s.pose = p
            fixed.append(s)
        fixed.append(raw[-1])
        return fixed

    def coverage_result_callback(self, future):
        result = future.result().result
        nav_path = result.nav_path
        nav_path.header.frame_id = 'map'
        if not nav_path.poses:
            self.get_logger().error("Coverage server returned an EMPTY path.")
            return

        self.get_logger().info(f"Got path with {len(nav_path.poses)} poses.")
        extracted = self.extract_waypoints(nav_path)
        
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.poses = [p.pose for p in extracted]

        self.final_path, self.final_waypoints = nav_path, pose_array
        now = self.get_clock().now().to_msg()
        nav_path.header.stamp, pose_array.header.stamp = now, now
        self.path_pub.publish(nav_path)
        self.waypoint_pub.publish(pose_array)

        if RVIZ_PREVIEW_ONLY:
            self.get_logger().info("=== RVIZ PREVIEW ONLY -- robot will NOT move. ===")
        else:
            # We pass the 'extracted' PoseStamped list, not just raw Poses
            self._start_hardware_execution(extracted)

    def _start_hardware_execution(self, poses):
        self.pose_queue = list(poses)
        self.get_logger().info(f"Starting hardware execution for {len(self.pose_queue)} waypoints.")
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if not self.pose_queue:
            self.get_logger().info("CLEANING CYCLE COMPLETE!")
            return
        
        if self.watchdog:
            self.watchdog.cancel()
            
        # Get the next PoseStamped from the queue
        target_stamped = self.pose_queue.pop(0)
        
        # ENSURE HEADER IS PRESENT (Fixes the transform error)
        target_stamped.header.frame_id = 'map'
        target_stamped.header.stamp = self.get_clock().now().to_msg()

        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_stamped # Action expects PoseStamped
        
        self.get_logger().info(f"Moving to next waypoint. ({len(self.pose_queue)} left)")
        self.watchdog = self.create_timer(60.0, self._watchdog_callback)
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self._nav_response_callback)

    def _nav_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Waypoint rejected, trying next.")
            self._send_next_waypoint()
            return
        self.current_goal_handle = gh
        gh.get_result_async().add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        if self.watchdog:
            self.watchdog.cancel()
        self.get_logger().info(f"Waypoint reached.")
        self._send_next_waypoint()

    def _watchdog_callback(self):
        self.get_logger().error("TIMEOUT: Robot stuck? Skipping current waypoint.")
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
        if self.watchdog:
            self.watchdog.cancel()

def main():
    rclpy.init()
    node = AutoCoverageExecute()
    node.process_and_send()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()