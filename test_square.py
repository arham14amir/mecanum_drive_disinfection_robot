import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
import time

from opennav_coverage_msgs.action import ComputeCoveragePath 
from opennav_coverage_msgs.msg import Coordinates, Coordinate

class ObstacleCoverageTest(Node):
    def __init__(self):
        super().__init__('obstacle_coverage_test')
        self.client = ActionClient(self, ComputeCoveragePath, 'compute_coverage_path')
        self.vis_pub = self.create_publisher(Path, '/test_coverage_path', 10)

    def send_goal(self):
        goal_msg = ComputeCoveragePath.Goal()

        # --- 1. THE ROOM (Outer Boundary) ---
        # 5x5 meters
        r1 = Coordinate(axis1=0.0, axis2=0.0)
        r2 = Coordinate(axis1=5.0, axis2=0.0)
        r3 = Coordinate(axis1=5.0, axis2=5.0)
        r4 = Coordinate(axis1=0.0, axis2=5.0)

        outer_boundary = Coordinates()
        outer_boundary.coordinates = [r1, r2, r3, r4, r1] # Closed Loop

        # --- 2. THE OBSTACLE (Inner Hole) ---
        # A 1x1 meter box in the center (from 2.0 to 3.0)
        o1 = Coordinate(axis1=2.0, axis2=2.0)
        o2 = Coordinate(axis1=3.0, axis2=2.0)
        o3 = Coordinate(axis1=3.0, axis2=3.0)
        o4 = Coordinate(axis1=2.0, axis2=3.0)

        obstacle = Coordinates()
        obstacle.coordinates = [o1, o2, o3, o4, o1] # Closed Loop

        # --- 3. Add BOTH to the Goal ---
        # List Item 0 = Boundary
        # List Item 1 = Obstacle
        goal_msg.polygons = [outer_boundary, obstacle]
        
        self.get_logger().info("Waiting for Coverage Server...")
        self.client.wait_for_server()
        
        self.get_logger().info("Sending Room with Obstacle...")
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted. Calculating around obstacle...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        nav_path = result.nav_path
        
        self.get_logger().info(f"Received path with {len(nav_path.poses)} points.")

        nav_path.header.frame_id = "map"
        
        for i in range(10):
            self.vis_pub.publish(nav_path)
            self.get_logger().info(f"Published to /test_coverage_path (Attempt {i+1})")
            time.sleep(1.0)
            
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ObstacleCoverageTest()
    node.send_goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()