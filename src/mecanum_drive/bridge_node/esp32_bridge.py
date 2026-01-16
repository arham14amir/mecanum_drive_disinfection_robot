#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import requests
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # === CONFIG ===
        # CHECK YOUR SERIAL MONITOR FOR THE REAL IP!
        self.esp32_ip = "10.180.73.210"   
        
        # FIX 1: Correct URL endpoints to match ESP32 code
        self.status_url = f"http://{self.esp32_ip}/status"
        self.cmd_url = f"http://{self.esp32_ip}/command" 

        # FIX 2: Updated Ticks to match your gear ratio (110:1)
        self.TICKS_PER_REV = 4840  

        # ROS publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for polling ESP32 status
        self.create_timer(0.1, self.poll_status)  # 10 Hz

        self.get_logger().info(f"ESP32 Bridge started, targeting {self.esp32_ip}")

    # === Handle incoming velocity commands ===
    def cmd_vel_callback(self, msg: Twist):
        try:
            # FIX 3: Format command exactly as ESP32 expects: "vx,vy,wz"
            # We limit decimals to 3 places to save URL length
            cmd_str = f"{msg.linear.x:.3f},{msg.linear.y:.3f},{msg.angular.z:.3f}"
            
            # Send as ?cmd=...
            payload = {'cmd': cmd_str}
            requests.get(self.cmd_url, params=payload, timeout=5.0)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to send command: {e}")

    # === Poll ESP32 status and publish odometry + joint states ===
    def poll_status(self):
        try:
            resp = requests.get(self.status_url, timeout=5.0)
            if resp.status_code != 200:
                return

            status_line = resp.text.strip()
            
            # Parse status: "odom_x=0.123 odom_y=0.456 ticksFL=123 ..."
            data = {}
            for kv in status_line.split():
                if '=' in kv:
                    k, v = kv.split("=")
                    try:
                        data[k] = float(v)
                    except ValueError:
                        data[k] = 0.0

            # --- Extract odometry ---
            x = data.get("odom_x", 0.0)
            y = data.get("odom_y", 0.0)
            theta = data.get("odom_theta", 0.0)
            vx = data.get("vx", 0.0)
            vy = data.get("vy", 0.0)
            wz = data.get("wz", 0.0)

            # --- Extract wheel ticks ---
            ticksFL = int(data.get("ticksFL", 0))
            ticksFR = int(data.get("ticksFR", 0))
            ticksRL = int(data.get("ticksRL", 0))
            ticksRR = int(data.get("ticksRR", 0))

            # --- Convert ticks to wheel angles ---
            theta_FL = ticksFL * 2 * pi / self.TICKS_PER_REV
            theta_FR = ticksFR * 2 * pi / self.TICKS_PER_REV
            theta_RL = ticksRL * 2 * pi / self.TICKS_PER_REV
            theta_RR = ticksRR * 2 * pi / self.TICKS_PER_REV

            # --- Publish JointState ---
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ["front_left_wheel_joint", "front_right_wheel_joint",
                              "rear_left_wheel_joint", "rear_right_wheel_joint"]
            joint_msg.position = [theta_FL, theta_FR, theta_RL, theta_RR]
            self.joint_pub.publish(joint_msg)

            # --- Publish Odometry ---
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0

            q = quaternion_from_euler(0, 0, theta)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = wz

            self.odom_pub.publish(odom)

            # --- Broadcast TF ---
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
