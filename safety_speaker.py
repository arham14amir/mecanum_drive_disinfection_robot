import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import subprocess
import time  # WE ADDED THIS BACK!

class SafetySpeakerNode(Node):
    def __init__(self):
        super().__init__('safety_speaker')
        
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # Settings
        self.danger_distance = 0.60
        self.cone_angle = 0.35
        self.audio_file = "/root/mecanum_drive_2/final_one.wav"

        # State tracking
        self.driving_straight = False
        self.is_yelling = False         # NEW: Tracks if we are actively yelling
        self.last_obstacle_time = 0.0   # NEW: Helps us ignore LiDAR noise
        self.audio_process = None
        
        self.get_logger().info("Smart Looping Safety Speaker Node Started!")

    def cmd_callback(self, msg):
        # We only care if the robot is generally trying to move forward
        if msg.linear.x > 0.05 and abs(msg.angular.z) < 0.20:
            self.driving_straight = True
        else:
            self.driving_straight = False

    def scan_callback(self, msg):
        min_distance = float('inf')
        
        for i, range_val in enumerate(msg.ranges):
            if math.isinf(range_val) or math.isnan(range_val) or range_val < msg.range_min:
                continue
                
            angle = msg.angle_min + (i * msg.angle_increment)
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle) < self.cone_angle:
                if range_val < min_distance:
                    min_distance = range_val

        # If we see an obstacle, record the exact time we saw it
        if min_distance < self.danger_distance:
            self.last_obstacle_time = time.time()

        self.manage_audio()

    def manage_audio(self):
        # 1-Second Debounce: Consider the path blocked if we saw an obstacle in the last 1.0 seconds
        obstacle_present = (time.time() - self.last_obstacle_time) < 1.0

        # DECISION LOGIC: 
        # Start yelling IF we are driving straight AND an obstacle appears.
        # KEEP yelling IF we are already yelling (even if Nav2 slams the brakes).
        if obstacle_present and (self.driving_straight or self.is_yelling):
            self.is_yelling = True
            
            if self.audio_process is None or self.audio_process.poll() is not None:
                self.get_logger().info("Obstacle blocking! Playing warning...")
                self.audio_process = subprocess.Popen(["/usr/bin/aplay", "-D", "plughw:2,0", self.audio_file])
                
        else:
            # If the path is clear (or we are turning a corner and never started yelling)
            if self.is_yelling:
                self.get_logger().info("Path cleared. Stopping audio.")
                self.is_yelling = False
            
            self.stop_audio()

    def stop_audio(self):
        if self.audio_process is not None:
            if self.audio_process.poll() is None: # Only terminate if it's actually still running
                self.audio_process.terminate()
            self.audio_process = None

def main(args=None):
    rclpy.init(args=args)
    node = SafetySpeakerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_audio()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()