#!/usr/bin/env python3
import math

import rclpy
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SpawnObstacleAhead(Node):
    def __init__(self):
        super().__init__('spawn_obstacle_ahead')
        self.declare_parameter('entity_name', 'temporary_box')
        self.declare_parameter('model_file', '')
        self.declare_parameter('ahead_distance', 1.2)
        self.declare_parameter('lateral_offset', 0.0)
        self.declare_parameter('z', 0.1)
        self.declare_parameter('wait_timeout_sec', 12.0)

        self.entity_name = self.get_parameter('entity_name').value
        self.model_file = self.get_parameter('model_file').value
        self.ahead_distance = float(self.get_parameter('ahead_distance').value)
        self.lateral_offset = float(self.get_parameter('lateral_offset').value)
        self.z = float(self.get_parameter('z').value)
        self.wait_timeout_sec = float(self.get_parameter('wait_timeout_sec').value)

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.odom_msg = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.2, self._tick)
        self.sent = False

    def _on_odom(self, msg: Odometry):
        self.odom_msg = msg

    @staticmethod
    def _yaw_from_quat(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _tick(self):
        if self.sent:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.wait_timeout_sec:
            self.get_logger().error('Timeout waiting for /odom pose.')
            rclpy.shutdown()
            return
        if self.odom_msg is None:
            return
        if not self.spawn_client.wait_for_service(timeout_sec=0.2):
            return
        if not self.model_file:
            self.get_logger().error('model_file is empty.')
            rclpy.shutdown()
            return

        pose = self.odom_msg.pose.pose
        yaw = self._yaw_from_quat(pose.orientation.z, pose.orientation.w)

        ahead_x = pose.position.x + self.ahead_distance * math.cos(yaw) - self.lateral_offset * math.sin(yaw)
        ahead_y = pose.position.y + self.ahead_distance * math.sin(yaw) + self.lateral_offset * math.cos(yaw)

        with open(self.model_file, 'r', encoding='utf-8') as f:
            model_xml = f.read()

        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = model_xml
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose.position.x = ahead_x
        req.initial_pose.position.y = ahead_y
        req.initial_pose.position.z = self.z

        self.get_logger().info(
            f'Spawning {self.entity_name} ahead of robot at '
            f'({ahead_x:.2f}, {ahead_y:.2f}, {self.z:.2f}), yaw={yaw:.2f}'
        )
        self.sent = True
        future = self.spawn_client.call_async(req)
        future.add_done_callback(self._on_spawn_done)

    def _on_spawn_done(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'Spawn result: success={res.success}, msg={res.status_message}')
        except Exception as exc:
            self.get_logger().error(f'Spawn call failed: {exc}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SpawnObstacleAhead()
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
