#!/usr/bin/env python3
"""Pubblica i waypoint del patrol come marker RViz su /argo_waypoints (sfera + nome)."""
import rclpy
import yaml
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class WaypointMarkers(Node):

    def __init__(self):
        super().__init__('waypoint_markers')
        self.declare_parameter('waypoints_file', '')
        path = self.get_parameter('waypoints_file').value
        with open(path) as f:
            waypoints = yaml.safe_load(f)['waypoints']

        self.markers = MarkerArray()
        for i, (name, wp) in enumerate(waypoints.items()):
            self.markers.markers.append(
                self.make_marker(2 * i, Marker.SPHERE, wp, 0.1, (1.0, 0.0, 0.0)))
            text = self.make_marker(2 * i + 1, Marker.TEXT_VIEW_FACING, wp, 0.7, (0.0, 0.0, 0.0))
            text.text = name
            self.markers.markers.append(text)

        self.publisher = self.create_publisher(MarkerArray, '/argo_waypoints', 10)
        # RViz può aprirsi in qualsiasi momento: ripubblico ogni secondo
        self.create_timer(1.0, lambda: self.publisher.publish(self.markers))
        self.get_logger().info(f'Pubblico {len(waypoints)} waypoint su /argo_waypoints')

    @staticmethod
    def make_marker(marker_id, marker_type, wp, z, rgb):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'waypoints'
        marker.id = marker_id
        marker.type = marker_type
        marker.pose.position.x = float(wp['x'])
        marker.pose.position.y = float(wp['y'])
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.35
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
