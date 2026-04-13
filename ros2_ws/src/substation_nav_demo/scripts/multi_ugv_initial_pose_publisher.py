#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Dict

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


@dataclass
class InitialPose:
    x: float
    y: float
    yaw: float


class MultiUgvInitialPosePublisher(Node):
    def __init__(self):
        super().__init__('multi_ugv_initial_pose_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_count', 12)
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('ugv1_x', -35.0)
        self.declare_parameter('ugv1_y', -35.0)
        self.declare_parameter('ugv1_yaw', 0.0)
        self.declare_parameter('ugv2_x', 45.0)
        self.declare_parameter('ugv2_y', 40.0)
        self.declare_parameter('ugv2_yaw', 0.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.remaining_publish_count = int(self.get_parameter('publish_count').value)
        period = float(self.get_parameter('publish_period_sec').value)

        self.poses: Dict[str, InitialPose] = {
            'ugv1': InitialPose(
                x=float(self.get_parameter('ugv1_x').value),
                y=float(self.get_parameter('ugv1_y').value),
                yaw=float(self.get_parameter('ugv1_yaw').value),
            ),
            'ugv2': InitialPose(
                x=float(self.get_parameter('ugv2_x').value),
                y=float(self.get_parameter('ugv2_y').value),
                yaw=float(self.get_parameter('ugv2_yaw').value),
            ),
        }
        self.publishers = {
            robot: self.create_publisher(PoseWithCovarianceStamped, f'/{robot}/initialpose', 10)
            for robot in self.poses.keys()
        }
        self.timer = self.create_timer(period, self._publish_tick)
        self.get_logger().info(
            f'Initial pose publisher started, frame={self.frame_id}, repeats={self.remaining_publish_count}'
        )

    def _publish_tick(self):
        if self.remaining_publish_count <= 0:
            self.timer.cancel()
            self.get_logger().info('Initial poses published, stopping repeats.')
            return

        now_msg = self.get_clock().now().to_msg()
        for robot, pose in self.poses.items():
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = now_msg
            msg.header.frame_id = self.frame_id
            msg.pose.pose.position.x = pose.x
            msg.pose.pose.position.y = pose.y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.z = math.sin(pose.yaw * 0.5)
            msg.pose.pose.orientation.w = math.cos(pose.yaw * 0.5)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.06853891909122467
            self.publishers[robot].publish(msg)

        self.remaining_publish_count -= 1


def main():
    rclpy.init()
    node = MultiUgvInitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
