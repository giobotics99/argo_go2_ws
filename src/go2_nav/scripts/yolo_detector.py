#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 YOLO Detection Node for Jetson Orin (GPU Accelerated).

Subscribes to /camera/color/image_raw, runs YOLOv8 Nano inference on CUDA,
and publishes detected object labels as a comma-separated string to /yolo/detections.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

# Import Ultralytics YOLO (GPU accelerated)
try:
    from ultralytics import YOLO
except ImportError:
    print("[ERROR] Ultralytics is not installed. Run: pip install ultralytics")
    raise

class YoloDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("model_name", "yolov8n.pt")  # Nano model (fastest on Jetson)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("inference_rate", 5.0)  # Throttled to 5 Hz to save GPU
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("use_cuda", True)

        self._model_name = self.get_parameter("model_name").value
        self._image_topic = self.get_parameter("image_topic").value
        self._detections_topic = self.get_parameter("detections_topic").value
        self._inference_rate = self.get_parameter("inference_rate").value
        self._conf_threshold = self.get_parameter("confidence_threshold").value
        self._use_cuda = self.get_parameter("use_cuda").value

        # ── CvBridge & YOLO Model ─────────────────────────────────────
        self._bridge = CvBridge()
        
        device = "cuda" if self._use_cuda else "cpu"
        self.get_logger().info(f"Loading YOLO model '{self._model_name}' on device '{device}'...")
        
        # This will load or download the model file to the current directory
        self._model = YOLO(self._model_name)
        self._model.to(device)
        
        # Warmup inference
        self.get_logger().info("Warming up GPU engine...")
        import numpy as np
        dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self._model(dummy_img, verbose=False, device=device)
        self.get_logger().info("YOLO Model loaded and warmed up successfully.")

        # ── State ─────────────────────────────────────────────────────
        self._last_inference_time = 0.0
        self._min_interval = 1.0 / self._inference_rate

        # ── ROS Interfaces ────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        
        self._img_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image_received,
            sensor_qos
        )
        
        self._detections_pub = self.create_publisher(
            String,
            self._detections_topic,
            10
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info("  YOLO Detector Node – ONLINE")
        self.get_logger().info(f"  Subscribing to : {self._image_topic}")
        self.get_logger().info(f"  Publishing to  : {self._detections_topic}")
        self.get_logger().info(f"  Target Rate    : {self._inference_rate} Hz")
        self.get_logger().info("=" * 60)

    def _on_image_received(self, msg: Image) -> None:
        now = time.monotonic()
        # Throttling inference to save GPU resources
        if now - self._last_inference_time < self._min_interval:
            return
        
        self._last_inference_time = now

        try:
            # Convert ROS Image to OpenCV BGR
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image message: {e}")
            return

        try:
            # Run inference on GPU
            device = "cuda" if self._use_cuda else "cpu"
            results = self._model(cv_img, conf=self._conf_threshold, verbose=False, device=device)
            
            if not results:
                return

            # Extract unique class names detected in the frame
            detected_classes = []
            for box in results[0].boxes:
                class_id = int(box.cls[0].item())
                class_name = self._model.names[class_id]
                detected_classes.append(class_name)

            # De-duplicate names
            unique_classes = list(set(detected_classes))

            # Publish as a comma-separated string
            out_msg = String()
            out_msg.data = ",".join(unique_classes)
            self._detections_pub.publish(out_msg)

            self.get_logger().debug(f"Published detections: {out_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error during YOLO inference: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
