#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import torch
import numpy as np
import math
import struct
import sys
import ctypes

class PointCloudFilterNode(Node):
    def __init__(self):
        super().__init__('pointcloud_filter_node')

        # Declare parameters
        self.declare_parameter("input_topic", "/lidar_points")
        self.declare_parameter("output_topic", "/lidar_points_filtered")
        self.declare_parameter("bounding_box_min", [-0.5, -0.4, -0.2])
        self.declare_parameter("bounding_box_max", [0.5, 0.4, 0.5])
        self.declare_parameter("enable_downsampling", False)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("enable_height_filter", False)
        self.declare_parameter("min_height", -float('inf'))
        self.declare_parameter("max_height", float('inf'))
        self.declare_parameter("enable_radius_filter", False)
        self.declare_parameter("max_radius", float('inf'))

        # Get parameters
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        
        bbox_min_list = self.get_parameter("bounding_box_min").value
        bbox_max_list = self.get_parameter("bounding_box_max").value
        
        # Check device
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"Using device: {self.device}")

        # Parameter setup
        self.bbox_min = torch.tensor(bbox_min_list, device=self.device, dtype=torch.float32)
        self.bbox_max = torch.tensor(bbox_max_list, device=self.device, dtype=torch.float32)

        self.enable_downsampling = self.get_parameter("enable_downsampling").value
        self.voxel_size = self.get_parameter("voxel_size").value
        self.enable_height_filter = self.get_parameter("enable_height_filter").value
        self.min_height = self.get_parameter("min_height").value
        self.max_height = self.get_parameter("max_height").value
        self.enable_radius_filter = self.get_parameter("enable_radius_filter").value
        self.max_radius = self.get_parameter("max_radius").value

        self.max_radius_sq = self.max_radius ** 2

        # Input QoS - Best Effort to match Lidar drivers
        input_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Output QoS - Reliable to match SLAM/Navigation defaults
        output_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.pc_callback, input_qos)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, output_qos)

        self.get_logger().info(f"Filtering pointcloud from {self.input_topic} to {self.output_topic}")
        self.get_logger().info("Note: Overwriting timestamps to current system time to fix sync issues.")
        
        self._cached_fields = None
        self._cached_offsets = None
        self._cached_point_step = None

    def create_cloud_xyz32(self, header, points):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_dense = False
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def read_points_numpy_fast(self, msg):
        if msg.point_step != self._cached_point_step or msg.fields != self._cached_fields:
            self._cached_point_step = msg.point_step
            self._cached_fields = msg.fields
            x_offset = y_offset = z_offset = -1
            for f in msg.fields:
                if f.name == 'x': x_offset = f.offset
                elif f.name == 'y': y_offset = f.offset
                elif f.name == 'z': z_offset = f.offset
            if x_offset == -1 or y_offset == -1 or z_offset == -1:
                return None, "Missing fields"
            self._cached_offsets = (x_offset, y_offset, z_offset)
            
        x_off, y_off, z_off = self._cached_offsets
        n_points = msg.width * msg.height
        if n_points == 0: return np.empty((0, 3), dtype=np.float32), None

        try:
            buf = memoryview(msg.data)
            def make_view(offset):
                return np.ndarray(shape=(n_points,), dtype=np.float32, buffer=buf, offset=offset, strides=(msg.point_step,))
            return np.stack((make_view(x_off), make_view(y_off), make_view(z_off)), axis=-1), None
        except Exception as e:
            return None, str(e)

    def pc_callback(self, msg):
        points_np, error = self.read_points_numpy_fast(msg)
        if error or points_np.shape[0] == 0: return

        points = torch.from_numpy(points_np).to(self.device)
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        
        in_box = (x >= self.bbox_min[0]) & (x <= self.bbox_min[0]) & \
                 (y >= self.bbox_min[1]) & (y <= self.bbox_min[1]) & \
                 (z >= self.bbox_min[2]) & (z <= self.bbox_min[2])
        
        # Correzione logica in_box per evitare errori di indice
        in_box = (x >= self.bbox_min[0]) & (x <= self.bbox_max[0]) & \
                 (y >= self.bbox_min[1]) & (y <= self.bbox_max[1]) & \
                 (z >= self.bbox_min[2]) & (z <= self.bbox_max[2])

        keep_mask = ~in_box
        
        if self.enable_height_filter:
            keep_mask &= (z >= self.min_height) & (z <= self.max_height)
        if self.enable_radius_filter:
            keep_mask &= (x.square() + y.square() <= self.max_radius_sq)
            
        points = points[keep_mask]
        
        # Create output message
        out_msg = self.create_cloud_xyz32(msg.header, points.cpu().numpy() if points.size(0) > 0 else np.empty((0, 3)))
        
        # --- FIX: Overwrite timestamp to current system time to solve the 2019 vs 2026 issue ---
        out_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
