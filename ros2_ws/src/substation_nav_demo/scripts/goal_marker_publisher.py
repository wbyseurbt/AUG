#!/usr/bin/env python3
import math
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
        _ = facility_csv
        markers: List[Tuple[str, float, float, float]] = [
            ('START', start_x, start_y, 0.0),
            ('ROAD_CRUISE_1', -30.0, -15.0, 0.0),
            ('tf_4', -13.2, -10.0, 0.0),
            ('tf_1', -13.2, 0.0, 0.0),
            ('tf_2', -13.2, 10.0, 0.0),
            ('tf_3', -13.2, 20.0, 0.0),
            ('ROAD_RETURN', -13.2, 35.0, 0.0),
            ('ROAD_RETURN2', 20.0, 35.0, 0.0),
            ('ins_row_1_post_3', 23.8, 20.0, 0.0),
            ('ins_row_2_post_3', 23.8, 0.0, 0.0),
            ('ins_row_3_post_3', 23.8, -20.0, 0.0),
            ('ins_row_6_post_3', 43.8, -20.0, 0.0),
            ('ins_row_5_post_3', 43.8, 0.0, 0.0),
            ('ins_row_4_post_3', 43.8, 20.0, 0.0),
            ('END', end_x, end_y, 0.0),
        ]

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
