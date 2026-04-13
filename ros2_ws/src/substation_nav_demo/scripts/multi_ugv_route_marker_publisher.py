#!/usr/bin/env python3
import math
from typing import Dict, List

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class MultiUgvRouteMarkerPublisher(Node):
    def __init__(self):
        super().__init__('multi_ugv_route_marker_publisher')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')

        self.frame_id = str(self.get_parameter('frame_id').value)
        waypoints_file = str(self.get_parameter('waypoints_file').value)
        self.routes = self._load_routes(waypoints_file)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(MarkerArray, '/multi_ugv_route_markers', qos)
        self.timer = self.create_timer(0.5, self._publish)
        self.get_logger().info(f'Publishing multi-UGV route markers: {list(self.routes.keys())}')

    def _load_routes(self, file_path: str) -> Dict[str, List[dict]]:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return data.get('robots', {})

    def _robot_color(self, robot: str):
        if robot == 'ugv1':
            return (1.0, 0.2, 0.2)
        return (0.2, 1.0, 0.2)

    def _publish(self):
        msg = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        for robot, points in self.routes.items():
            if not points:
                continue

            r, g, b = self._robot_color(robot)

            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = stamp
            line.ns = f'{robot}_route_line'
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color.a = 0.95
            line.color.r = r
            line.color.g = g
            line.color.b = b
            for p in points:
                pt = Point()
                pt.x = float(p.get('x', 0.0))
                pt.y = float(p.get('y', 0.0))
                pt.z = 0.1
                line.points.append(pt)
            msg.markers.append(line)

            for i, p in enumerate(points):
                x = float(p.get('x', 0.0))
                y = float(p.get('y', 0.0))
                name = str(p.get('name', f'P{i + 1}'))

                arrow = Marker()
                arrow.header.frame_id = self.frame_id
                arrow.header.stamp = stamp
                arrow.ns = f'{robot}_route_arrow'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = x
                arrow.pose.position.y = y
                arrow.pose.position.z = 0.08
                if i + 1 < len(points):
                    nx = float(points[i + 1].get('x', x))
                    ny = float(points[i + 1].get('y', y))
                    yaw = math.atan2(ny - y, nx - x)
                else:
                    yaw = 0.0
                arrow.pose.orientation.z = math.sin(yaw * 0.5)
                arrow.pose.orientation.w = math.cos(yaw * 0.5)
                arrow.scale.x = 0.7
                arrow.scale.y = 0.12
                arrow.scale.z = 0.12
                arrow.color.a = 1.0
                arrow.color.r = r
                arrow.color.g = g
                arrow.color.b = b
                msg.markers.append(arrow)

                text = Marker()
                text.header.frame_id = self.frame_id
                text.header.stamp = stamp
                text.ns = f'{robot}_route_text'
                text.id = marker_id
                marker_id += 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x
                text.pose.position.y = y
                text.pose.position.z = 0.8
                text.scale.z = 0.28
                text.color.a = 1.0
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 0.2
                text.text = f'{robot}:{i + 1}:{name}'
                msg.markers.append(text)

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MultiUgvRouteMarkerPublisher()
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
