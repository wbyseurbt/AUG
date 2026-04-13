#!/usr/bin/env python3
import csv
import math
import re
from typing import List, Tuple

import rclpy
from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class GoalMarkerPublisher(Node):
    def __init__(self):
        super().__init__('goal_marker_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('facility_csv', '')
        self.declare_parameter('route_start_x', -35.0)
        self.declare_parameter('route_start_y', -35.0)
        self.declare_parameter('route_end_x', 45.0)
        self.declare_parameter('route_end_y', 40.0)

        self.frame_id = self.get_parameter('frame_id').value
        facility_csv = str(self.get_parameter('facility_csv').value)
        route_start_x = float(self.get_parameter('route_start_x').value)
        route_start_y = float(self.get_parameter('route_start_y').value)
        route_end_x = float(self.get_parameter('route_end_x').value)
        route_end_y = float(self.get_parameter('route_end_y').value)

        self.goals = self._build_markers_from_csv(
            facility_csv=facility_csv,
            start_x=route_start_x,
            start_y=route_start_y,
            end_x=route_end_x,
            end_y=route_end_y,
        )

        # TRANSIENT_LOCAL：RViz 后启动仍能收到上一帧；Reliable 与 RViz Marker 订阅匹配
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(MarkerArray, '/goal_markers', marker_qos)
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.get_logger().info(f'Publishing {len(self.goals)} goal markers on /goal_markers')

    def _build_markers_from_csv(
        self,
        facility_csv: str,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
    ) -> List[Tuple[str, float, float, float]]:
        markers: List[Tuple[str, float, float, float]] = [
            ('START', start_x, start_y, 0.0),
            ('ROAD_CRUISE_1', -30.0, -15.0, 0.0),
        ]
        transformers = {}
        ins_rows = {}

        if facility_csv:
            with open(facility_csv, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    name = (row.get('name') or '').strip()
                    if '//' in name:
                        name = name.split('//', 1)[0].strip()
                    lower_name = name.lower()
                    if not lower_name or 'wall' in lower_name:
                        continue
                    try:
                        x = float((row.get('x') or '').strip())
                        y = float((row.get('y') or '').strip())
                    except (TypeError, ValueError):
                        continue

                    tf_match = re.fullmatch(r'tf_(\d+)', lower_name)
                    if tf_match:
                        transformers[int(tf_match.group(1))] = (name, x, y)
                        continue

                    row_match = re.fullmatch(r'ins_row_(\d+)_post_(\d+)', lower_name)
                    if row_match:
                        row_idx = int(row_match.group(1))
                        post_idx = int(row_match.group(2))
                        ins_rows.setdefault(row_idx, []).append((post_idx, name, x, y))

        for tf_idx in (1, 2, 3, 4):
            if tf_idx in transformers:
                name, x, y = transformers[tf_idx]
                markers.append((name, x, y, 0.0))

        markers.append(('ROAD_RETURN', 8.0, 0.0, 0.0))

        for row_idx in range(1, 7):
            posts = ins_rows.get(row_idx, [])
            if not posts:
                continue
            posts.sort(key=lambda item: item[0])
            target = None
            for post_idx, post_name, x, y in posts:
                if post_idx == 3:
                    target = (post_name, x, y)
                    break
            if target is None:
                _, post_name, x, y = posts[len(posts) // 2]
                target = (post_name, x, y)
            name, x, y = target
            markers.append((name, x, y, 0.0))

        markers.append(('END', end_x, end_y, 0.0))

        marker_list: List[Tuple[str, float, float, float]] = []
        for i, (name, x, y, _) in enumerate(markers):
            if i + 1 < len(markers):
                _, next_x, next_y, _ = markers[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = 0.0
            marker_list.append((name, x, y, yaw))
        return marker_list

    def publish_markers(self):
        marker_array = MarkerArray()
        # 时间戳置零：RViz2 在 use_sim_time 下用“最新 TF”，避免与仿真时钟不同步导致 Marker 不显示
        stamp = Time(sec=0, nanosec=0)
        infinite_life = Duration(sec=0, nanosec=0)

        for i, (name, x, y, yaw) in enumerate(self.goals):
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = stamp
            arrow.ns = 'goal_arrows'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.z = math.sin(yaw * 0.5)
            arrow.pose.orientation.w = math.cos(yaw * 0.5)
            arrow.scale.x = 0.8
            arrow.scale.y = 0.16
            arrow.scale.z = 0.16
            arrow.color.a = 1.0
            arrow.color.r = 1.0 if i == 0 else 0.2
            arrow.color.g = 0.2 if i == 0 else 1.0
            arrow.color.b = 0.2
            arrow.lifetime = infinite_life

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = stamp
            text.ns = 'goal_text'
            text.id = 100 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.7
            text.scale.z = 0.35
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 0.2
            text.text = f'{i + 1}:{name}'
            text.lifetime = infinite_life

            marker_array.markers.extend([arrow, text])

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = stamp
        line.ns = 'goal_path_hint'
        line.id = 999
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.06
        line.color.a = 0.8
        line.color.r = 0.2
        line.color.g = 0.8
        line.color.b = 1.0
        for _, x, y, _ in self.goals:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.1
            line.points.append(p)
        line.lifetime = infinite_life
        marker_array.markers.append(line)

        self.pub.publish(marker_array)


def main():
    rclpy.init()
    node = GoalMarkerPublisher()
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
