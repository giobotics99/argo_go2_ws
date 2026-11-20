#!/usr/bin/env python3
"""Pubblica i luoghi come marker RViz su /argo_waypoints (sfera + nome).

v1: parametro waypoints_file (config/waypoints.yaml): stanze in rosso.
v2: parametro ontology_file (config/ontology.yaml): stanze in rosso, hub del corridoio in blu,
    archi del grafo come linee grigie. Se waypoints_file è vuoto si usa ontology_file.
In RViz: Add -> By topic -> /argo_waypoints -> MarkerArray.
"""
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

RED, BLUE, BLACK, GREY = (1.0, 0.0, 0.0), (0.1, 0.3, 1.0), (0.0, 0.0, 0.0), (0.5, 0.5, 0.5)


class WaypointMarkers(Node):

    def __init__(self):
        super().__init__('waypoint_markers')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('ontology_file', os.path.join(
            get_package_share_directory('go2_patrol_mission'), 'config', 'ontology.yaml'))
        waypoints_file = self.get_parameter('waypoints_file').value
        self.markers = MarkerArray()
        self.next_id = 0

        if waypoints_file:
            with open(waypoints_file) as f:
                points = {n: (wp, RED) for n, wp in yaml.safe_load(f)['waypoints'].items()}
            edges = []
            source = waypoints_file
        else:
            source = self.get_parameter('ontology_file').value
            with open(source) as f:
                onto = yaml.safe_load(f)
            points = {n: (p['pose'], RED) for n, p in onto['places'].items()}
            points.update({n: (h, BLUE) for n, h in onto['hubs'].items()})
            edges = [(e[0], e[1]) for e in onto.get('edges', [])]

        for name, (pose, rgb) in points.items():
            self.add(self.make_marker(Marker.SPHERE, pose, 0.1, rgb, 0.35))
            text = self.make_marker(Marker.TEXT_VIEW_FACING, pose, 0.7, BLACK, 0.35)
            text.text = name
            self.add(text)
        if edges:
            line = self.make_marker(Marker.LINE_LIST, {'x': 0.0, 'y': 0.0}, 0.0, GREY, 0.05)
            line.pose.position.z = 0.05
            for a, b in edges:
                for n in (a, b):
                    pose = points[n][0]
                    line.points.append(Point(x=float(pose['x']), y=float(pose['y']), z=0.0))
            self.add(line)

        self.publisher = self.create_publisher(MarkerArray, '/argo_waypoints', 10)
        # RViz può aprirsi in qualsiasi momento: ripubblico ogni secondo
        self.create_timer(1.0, lambda: self.publisher.publish(self.markers))
        self.get_logger().info(f'Pubblico {len(points)} luoghi e {len(edges)} archi su /argo_waypoints ({source})')

    def add(self, marker):
        marker.id = self.next_id
        self.next_id += 1
        self.markers.markers.append(marker)

    @staticmethod
    def make_marker(marker_type, pose, z, rgb, scale):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'waypoints'
        marker.type = marker_type
        marker.pose.position.x = float(pose['x'])
        marker.pose.position.y = float(pose['y'])
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 1.0
        return marker


def main():
    rclpy.init()
    node = WaypointMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
