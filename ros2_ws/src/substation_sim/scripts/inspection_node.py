#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

class InspectionNode(Node):
    def __init__(self):
        super().__init__('inspection_node')
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Waypoints: (x, y, yaw)
        self.waypoints = [
            (10.0, 10.0, 0.0, "Transformer 1"),
            (10.0, -10.0, 1.57, "Transformer 2"),
            (20.0, 30.0, 0.0, "Tower 1"),
            (-20.0, -20.0, 0.0, "Home")
        ]
        self.current_wp_index = 0
        self.declare_parameter('screenshot_dir', '/tmp/inspection_results')
        self.screenshot_dir = self.get_parameter('screenshot_dir').value
        try:
            os.makedirs(self.screenshot_dir, exist_ok=True)
        except OSError:
            self.screenshot_dir = '/tmp/inspection_results'
            os.makedirs(self.screenshot_dir, exist_ok=True)
            
    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def send_goal(self, x, y, yaw, name):
        self.get_logger().info(f'Navigating to {name}...')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = yaw # Simplified yaw
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.nav_client.wait_for_server()
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.current_wp_name = name

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Reached {self.current_wp_name}!')
        self.perform_inspection()

    def perform_inspection(self):
        self.get_logger().info(f'正在检测设备 {self.current_wp_name}...')
        time.sleep(4) # Stay for 3-5 seconds
        
        if self.latest_image is not None:
            filename = os.path.join(self.screenshot_dir, f'detect_{self.current_wp_name}_{int(time.time())}.jpg')
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f'Screenshot saved: {filename}')
        
        self.current_wp_index += 1
        if self.current_wp_index < len(self.waypoints):
            next_wp = self.waypoints[self.current_wp_index]
            self.send_goal(*next_wp)
        else:
            self.get_logger().info('Inspection Task Completed!')

def main(args=None):
    rclpy.init(args=args)
    node = InspectionNode()
    # Start with first waypoint
    node.send_goal(*node.waypoints[0])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
