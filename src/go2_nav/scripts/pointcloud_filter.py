#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import math
import struct
import sys
import ctypes
import array

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
        # GPU is OFF by default: the mask on a single lidar scan is ~1 ms on the
        # Orin CPU, while a CUDA context costs ~0.5-1 GB of the unified memory
        # that llama.cpp (Gemma) and YOLO need. Enable only for very dense clouds.
        self.declare_parameter("use_gpu", False)

        # Get parameters
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        
        bbox_min_list = self.get_parameter("bounding_box_min").value
        bbox_max_list = self.get_parameter("bounding_box_max").value
        
        # Device selection (CPU numpy by default, optional torch/CUDA)
        self.torch = None
        self.device = None
        if self.get_parameter("use_gpu").value:
            try:
                import torch
                if torch.cuda.is_available():
                    self.device = torch.device('cuda')
                    torch.zeros(1, device=self.device)  # force context creation now
                    self.torch = torch
                else:
                    self.get_logger().warn("use_gpu=True but torch has no CUDA support: falling back to CPU")
            except Exception as e:
                self.get_logger().warn(f"use_gpu=True but CUDA init failed ({e}): falling back to CPU")
                self.torch = None
        self.get_logger().info(f"Using device: {'cuda (torch)' if self.torch else 'cpu (numpy)'}")

        # Parameter setup
        self.bbox_min = np.asarray(bbox_min_list, dtype=np.float32)
        self.bbox_max = np.asarray(bbox_max_list, dtype=np.float32)

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
        
        # array('B') hits rclpy's fast path (a bytes object is validated element by element)
        msg.data = array.array('B', points.astype(np.float32).tobytes())
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

    def _filter_mask_numpy(self, pts):
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        in_box = np.all((pts >= self.bbox_min) & (pts <= self.bbox_max), axis=1)
        keep = ~in_box & np.isfinite(pts).all(axis=1)
        if self.enable_height_filter:
            keep &= (z >= self.min_height) & (z <= self.max_height)
        if self.enable_radius_filter:
            keep &= (x * x + y * y) <= self.max_radius_sq
        return keep

    def _filter_torch(self, pts_np):
        torch = self.torch
        pts = torch.from_numpy(pts_np).to(self.device)
        bmin = torch.from_numpy(self.bbox_min).to(self.device)
        bmax = torch.from_numpy(self.bbox_max).to(self.device)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        in_box = ((pts >= bmin) & (pts <= bmax)).all(dim=1)
        keep = ~in_box & torch.isfinite(pts).all(dim=1)
        if self.enable_height_filter:
            keep &= (z >= self.min_height) & (z <= self.max_height)
        if self.enable_radius_filter:
            keep &= (x * x + y * y) <= self.max_radius_sq
        pts = pts[keep]
        if self.enable_downsampling:
            pts = pts[::4]
        return pts.cpu().numpy()

    def pc_callback(self, msg):
        try:
            # 1. Fast zero-copy read of x/y/z
            points_np, error = self.read_points_numpy_fast(msg)
            if error or points_np is None or points_np.shape[0] == 0:
                return

            # 2. Remove robot body (bounding box), NaNs, optional height/radius
            if self.torch is not None:
                try:
                    out_points_np = self._filter_torch(points_np)
                except Exception as e:
                    self.get_logger().error(f"CUDA filter failed ({e}); switching to CPU permanently")
                    self.torch = None
                    return
            else:
                out_points_np = points_np[self._filter_mask_numpy(points_np)]
                # 3. Light downsampling (1 point out of 4)
                if self.enable_downsampling:
                    out_points_np = out_points_np[::4]

            # 4. Publish
            out_msg = self.create_cloud_xyz32(msg.header, np.ascontiguousarray(out_points_np, dtype=np.float32))
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Errore nel callback di filtraggio: {e}")

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
