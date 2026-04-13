#!/usr/bin/env python3
import math
import random

import rclpy
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SpawnObstacleAhead(Node):
    def __init__(self):
        super().__init__('spawn_obstacle_ahead')
        self.declare_parameter('entity_name', 'temporary_box')
        self.declare_parameter('entity_prefix', 'temporary_box')
        self.declare_parameter('model_file', '')
        self.declare_parameter('ahead_distance', 1.2)
        self.declare_parameter('lateral_offset', 0.0)
        self.declare_parameter('random_lateral_range', 0.35)
        self.declare_parameter('z', 0.1)
        self.declare_parameter('start_delay_sec', 30.0)
        self.declare_parameter('spawn_period_sec', 60.0)
        self.declare_parameter('max_spawns', 0)
        self.declare_parameter('wait_timeout_sec', 12.0)

        self.entity_name = self.get_parameter('entity_name').value
        self.entity_prefix = self.get_parameter('entity_prefix').value
        self.model_file = self.get_parameter('model_file').value
        self.ahead_distance = float(self.get_parameter('ahead_distance').value)
        self.lateral_offset = float(self.get_parameter('lateral_offset').value)
        self.random_lateral_range = float(self.get_parameter('random_lateral_range').value)
        self.z = float(self.get_parameter('z').value)
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.spawn_period_sec = float(self.get_parameter('spawn_period_sec').value)
        self.max_spawns = int(self.get_parameter('max_spawns').value)
        self.wait_timeout_sec = float(self.get_parameter('wait_timeout_sec').value)

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.odom_msg = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.start_time = self.get_clock().now()
        self.next_spawn_time_sec = self.start_delay_sec
        self.spawn_count = 0
        self.model_xml = ''
        self.timer = self.create_timer(0.2, self._tick)

    def _on_odom(self, msg: Odometry):
        self.odom_msg = msg

    @staticmethod
    def _yaw_from_quat(z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

    def _tick(self):
        if self.max_spawns > 0 and self.spawn_count >= self.max_spawns:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.wait_timeout_sec and self.odom_msg is None:
            self.get_logger().error('Timeout waiting for /odom pose.')
            return
        if elapsed < self.next_spawn_time_sec:
            return
        if self.odom_msg is None:
            return
        if not self.spawn_client.wait_for_service(timeout_sec=0.2):
            return
        if not self.model_file:
            self.get_logger().error('model_file is empty.')
            return

        if not self.model_xml:
            with open(self.model_file, 'r', encoding='utf-8') as f:
                self.model_xml = f.read()

        pose = self.odom_msg.pose.pose
        yaw = self._yaw_from_quat(pose.orientation.z, pose.orientation.w)
        lateral = self.lateral_offset + random.uniform(-self.random_lateral_range, self.random_lateral_range)
        ahead_x = pose.position.x + self.ahead_distance * math.cos(yaw) - lateral * math.sin(yaw)
        ahead_y = pose.position.y + self.ahead_distance * math.sin(yaw) + lateral * math.cos(yaw)
        self.spawn_count += 1
        name = self.entity_name if self.spawn_period_sec <= 0.0 else f'{self.entity_prefix}_{self.spawn_count}'

        req = SpawnEntity.Request()
        req.name = name
        req.xml = self.model_xml
        req.robot_namespace = ''
        req.reference_frame = 'world'
        req.initial_pose.position.x = ahead_x
        req.initial_pose.position.y = ahead_y
        req.initial_pose.position.z = self.z

        self.get_logger().info(
            f'Spawning {name} ahead of robot at '
            f'({ahead_x:.2f}, {ahead_y:.2f}, {self.z:.2f}), yaw={yaw:.2f}, lateral={lateral:.2f}'
        )
        future = self.spawn_client.call_async(req)
        future.add_done_callback(self._on_spawn_done)
        if self.spawn_period_sec > 0.0:
            self.next_spawn_time_sec += self.spawn_period_sec

    def _on_spawn_done(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'Spawn result: success={res.success}, msg={res.status_message}')
        except Exception as exc:
            self.get_logger().error(f'Spawn call failed: {exc}')


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
