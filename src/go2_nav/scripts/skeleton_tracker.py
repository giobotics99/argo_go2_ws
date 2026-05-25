#!/usr/bin/python3

import os
import cv2
import numpy as np
import pyrealsense2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.duration import Duration
from rclpy.action import ActionClient

import tf2_ros
import tf2_geometry_msgs
import math

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo

# Importiamo l'azione corretta per Foxy
from nav2_msgs.action import NavigateToPose

import sys 
sys.path.append("/home/unitree/assistive_robodog/ROS/3d_human_detection")
from src.landmark_extractor import LandmarkExtractor

class SkeletonTracker3D(Node):

    def __init__(self):
        super().__init__('skeleton_tracker_node')
        self.br = CvBridge()
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_link')

        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sottoscrizioni tramite message_filters per sincronizzarle
        self.rgb_sub = Subscriber(self, Image, "/camera/color/image_raw", qos_profile=qos_profile)
        self.depth_sub = Subscriber(self, Image, "/camera/aligned_depth_to_color/image_raw", qos_profile=qos_profile)
        
        # Sincronizzatore: tolleranza di 0.05 secondi tra i due frame
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.synchronized_callback)
           
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.camera_info_sub_callback, qos_profile)

        self.skeleton_image_pub = self.create_publisher(Image, "/camera/skeleton_image", 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.human_marker_pub = self.create_publisher(Marker, 'human_marker_cl', 10)
        self.human_pose_pub = self.create_publisher(PoseStamped, '/detected_body_pose', 10)
        
        # Publisher per l'aggiornamento continuo (intercettato dal GoalUpdater di Nav2)
        self.goal_update_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Action Client per risvegliare l'albero di navigazione in Foxy
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.camera_info_received = False
        self._intrinsics = None
        self.landmark_extractor = LandmarkExtractor()

        self.prev_pose_global = None 
        self.current_yaw = 0.0       
        
        # Filtro per Nav2: memorizza l'ultimo goal effettivamente pubblicato
        self.last_sent_goal = None
        self.tree_started = False

        self.get_logger().info("Nodo SkeletonTracker3D avviato con Successo. Architettura Foxy: Action + GoalUpdater.")

    def camera_info_sub_callback(self, msg):
        if not self.camera_info_received:
            self.camera_info_received = True
            self._intrinsics = pyrealsense2.intrinsics()
            self._intrinsics.width = msg.width
            self._intrinsics.height = msg.height
            self._intrinsics.ppx = msg.k[2]
            self._intrinsics.ppy = msg.k[5]
            self._intrinsics.fx = msg.k[0]
            self._intrinsics.fy = msg.k[4]
            self._intrinsics.model = pyrealsense2.distortion.none     
            self._intrinsics.coeffs = [i for i in msg.d]
            self.get_logger().info("Parametri intrinseci della camera ricevuti correttamente.")

    def synchronized_callback(self, rgb_msg, depth_msg):
        """ Callback unico sincronizzato per RGB e Depth """
        if not self.camera_info_received:
            return

        timestamp = rgb_msg.header.stamp
        frame = self.br.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_map = self.br.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # Estrazione dei landmark
        img_landmarks, _ = self.landmark_extractor.get_body_landmarks(frame)

        image_out = frame.copy()

        if img_landmarks is not None:
            cx, cy = self.get_body_center(img_landmarks)

            window_size = 21
            half_w = window_size // 2

            x_start = max(0, cx - half_w)
            x_end = min(depth_map.shape[1], cx + half_w + 1)
            y_start = max(0, cy - half_w)
            y_end = min(depth_map.shape[0], cy + half_w + 1)

            roi = depth_map[y_start:y_end, x_start:x_end]
            valid_depths = roi[roi > 0]

            if valid_depths.size > 0:
                target_depth = np.percentile(valid_depths, 10)
                real_x, real_y, real_z = self.project_landmark_to_3D(cx, cy, target_depth)

                # Mappatura corretta per il frame standard ROS 'camera_link'
                cam_x = float(real_z) / 1000.0   # La profondità della telecamera diventa l'asse X (avanti)
                cam_y = float(-real_x) / 1000.0  # L'asse laterale invertito diventa l'asse Y (sinistra)
                cam_z = float(-real_y) / 1000.0  # L'altezza invertita diventa l'asse Z (alto)

                # Filtro di smoothing (Alpha-Beta)
                alpha = 0.2
                if not hasattr(self, 'smoothed_pos'):
                    self.smoothed_pos = [cam_x, cam_y, cam_z]
                else:
                    self.smoothed_pos[0] = alpha * cam_x + (1 - alpha) * self.smoothed_pos[0]
                    self.smoothed_pos[1] = alpha * cam_y + (1 - alpha) * self.smoothed_pos[1]
                    self.smoothed_pos[2] = alpha * cam_z + (1 - alpha) * self.smoothed_pos[2]

                # Disegna il pallino verde sul centro del corpo nell'immagine di output
                cv2.circle(image_out, (cx, cy), 10, (0, 255, 0), thickness=-1)
                
                # Invia il punto alla pipeline di calcolo del Goal
                self.visualize_points([self.smoothed_pos], timestamp)
        else:
            if hasattr(self, 'smoothed_pos'):
                del self.smoothed_pos
        
        self.skeleton_image_pub.publish(self.br.cv2_to_imgmsg(image_out, encoding="bgr8"))

    def project_landmark_to_3D(self, x, y, depth):
        if self._intrinsics is None:
            return 0.0, 0.0, 0.0
        return pyrealsense2.rs2_deproject_pixel_to_point(self._intrinsics, [x, y], depth)

    def get_body_center(self, landmarks):
        ls, rs = landmarks['left_shoulder'], landmarks['right_shoulder']
        lh, rh = landmarks['left_hip'], landmarks['right_hip']
        c_x = ((ls[0] + rs[0]) / 2 + (lh[0] + rh[0]) / 2) / 2
        c_y = ((ls[1] + lh[1]) / 2 + (rs[1] + rh[1]) / 2) / 2
        return int(c_x), int(c_y)

    def visualize_points(self, points, timestamp):
            if not points:
                return
            
            # 1. Pubblica il Marker originale per vedere l'umano su RViz nel frame ottico corretto
            marker = Marker()
            marker.header.frame_id = self.camera_frame
            marker.header.stamp = timestamp
            marker.ns = "points"
            marker.id = 0
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x, marker.scale.y = 0.1, 0.1
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            
            for pt in points:
                p = Point()
                p.x, p.y, p.z = pt[0], pt[1], pt[2]
                marker.points.append(p)
            self.human_marker_pub.publish(marker)

            try:
                # Recuperiamo le coordinate dell'umano dal frame ottico (X=Destra, Y=Basso, Z=Profondità)
                human_cam_x = points[0][0]
                human_cam_y = points[0][1]
                human_cam_z = points[0][2]
                
                # Calcoliamo la reale distanza euclidea 3D dall'ottica della telecamera
                distance_to_human = math.sqrt(human_cam_x**2 + human_cam_y**2 + human_cam_z**2)
                safe_distance = 0.4

                # Calcoliamo il GOAL riducendo il vettore direttamente nello spazio ottico della telecamera
                if distance_to_human > safe_distance:
                    # Accorciamo il vettore di 40cm mantenendo la stessa linea retta visiva
                    scale = (distance_to_human - safe_distance) / distance_to_human
                    goal_cam_x = human_cam_x * scale
                    goal_cam_y = human_cam_y * scale
                    goal_cam_z = human_cam_z * scale
                else:
                    # Se l'umano è già entro i 40cm, fermati sul posto
                    goal_cam_x = 0.0
                    goal_cam_y = 0.0
                    goal_cam_z = 0.0

                # 2. TRASFORMAZIONE TF: Trasformiamo il goal ottico in coordinate globali di mappa
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame, 
                    self.camera_frame, 
                    rclpy.time.Time(), 
                    timeout=Duration(seconds=0.1)
                )
                
                point_ = PointStamped()
                point_.header.frame_id = self.camera_frame
                point_.header.stamp = timestamp # Sincronizzato con il tempo dello scatto del frame
                point_.point.x = goal_cam_x
                point_.point.y = goal_cam_y
                point_.point.z = goal_cam_z # Inseriamo la Z ottica corretta prima della rotazione di TF
                
                point_in_global = tf2_geometry_msgs.do_transform_point(point_, transform)
                goal_x = point_in_global.point.x
                goal_y = point_in_global.point.y

                # 3. Controllo del filtro di invio per Nav2 (Banda morta di 25cm)
                should_send_goal = True
                if self.last_sent_goal is not None:
                    dist_from_last_goal = math.sqrt(
                        (goal_x - self.last_sent_goal.x)**2 + 
                        (goal_y - self.last_sent_goal.y)**2
                    )
                    if dist_from_last_goal < 0.25: 
                        should_send_goal = False

                if should_send_goal:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = self.global_frame
                    pose_msg.pose.position.x = goal_x
                    pose_msg.pose.position.y = goal_y
                    pose_msg.pose.position.z = 0.0 # Navigazione 2D (la mappa schiaccia a terra l'altezza)
                    
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = 0.0
                    pose_msg.pose.orientation.w = 1.0
                    
                    if not self.tree_started:
                        self.send_initial_action_goal(pose_msg)
                    else:
                        self.goal_update_pub.publish(pose_msg)
                        self.get_logger().info(f"Nuovo Goal proiettato (Globale): x={goal_x:.2f}, y={goal_y:.2f} (Distanza: {distance_to_human:.2f}m)")

                    self.last_sent_goal = pose_msg.pose.position 

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Failed to transform goal position: {str(e)}")
            
    def send_initial_action_goal(self, pose_msg):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Attesa Action Server 'navigate_to_pose'...")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self.get_logger().info("Innesco iniziale Nav2 tramite Action NavigateToPose...")
        self.nav_to_pose_client.send_goal_async(goal_msg)
        self.tree_started = True

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return qx, qy, qz, qw
    
def main(args=None):
    rclpy.init(args=args)
    node = SkeletonTracker3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()