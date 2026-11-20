#!/usr/bin/env python3
"""Record named poses on the map for ontology.yaml (tutorial step 1).

Press Enter, type a name, and the current robot pose is appended to tools/recorded_poses.yaml as
    name: {x: ..., y: ..., theta: ...}      (theta in radians)

The pose is read from the TF map -> base_link at the moment you press Enter: it is always up to
date, while AMCL publishes /amcl_pose only after the robot has moved (~0.25 m or a rotation).
/amcl_pose is used only as a liveness check: a warning is shown if it has been silent for 5 s.

Usage (pipeline running, 2D Pose Estimate given):
    source ~/argo_go2_ws/scripts/argo_env.sh
    python3 ~/argo_go2_ws/tools/record_pose.py            # or --file other.yaml
Type "q" to quit.
"""
import argparse
import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

AMCL_TIMEOUT = 5.0  # s without /amcl_pose before warning
DEFAULT_FILE = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'recorded_poses.yaml')


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseRecorder(Node):

    def __init__(self):
        super().__init__('argo_pose_recorder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # AMCL runs in the Humble container and publishes /amcl_pose as transient local
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, qos)
        self.last_amcl = None  # wall time of the last /amcl_pose
        self.warned = False
        self.create_timer(1.0, self.check_amcl)

    def on_amcl(self, _msg):
        self.last_amcl = time.time()
        if self.warned:
            print('\n[record_pose] /amcl_pose di nuovo attivo.', flush=True)
            self.warned = False

    def amcl_age(self):
        return None if self.last_amcl is None else time.time() - self.last_amcl

    def check_amcl(self):
        age = self.amcl_age()
        if (age is None or age > AMCL_TIMEOUT) and not self.warned:
            self.warned = True
            what = 'mai ricevuto' if age is None else f'fermo da {age:.0f} s'
            print(f'\n[record_pose] ATTENZIONE: /amcl_pose {what}. Normale se il robot è fermo; '
                  'se non arriva mai, controlla Nav2 e la 2D Pose Estimate.', flush=True)

    def current_pose(self):
        """Latest map -> base_link as (x, y, theta), or None if the TF is not available."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception as e:  # noqa: BLE001 - any TF error means "no pose"
            self.get_logger().warn(f'TF map -> base_link non disponibile: {e}')
            return None
        tr = t.transform.translation
        return round(tr.x, 3), round(tr.y, 3), round(yaw_from_quaternion(t.transform.rotation), 3)


def load_poses(path):
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def save_poses(path, poses):
    """One line per pose, in insertion order, easy to read and to diff."""
    lines = ['# Pose rilevate con tools/record_pose.py (frame map, theta in radianti)']
    for name, p in poses.items():
        lines.append(f"{name}: {{x: {p['x']}, y: {p['y']}, theta: {p['theta']}}}")
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description='Salva pose con nome dalla TF map -> base_link')
    parser.add_argument('--file', default=DEFAULT_FILE, help='file YAML di uscita')
    args = parser.parse_args()

    rclpy.init()
    node = PoseRecorder()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f'Salvo in {args.file}')
    print('Porta il robot nel punto, orientalo, poi premi Invio. "q" + Invio per uscire.')
    try:
        while True:
            if input('\n[Invio = salva, q = esci] ').strip().lower() == 'q':
                break
            pose = node.current_pose()
            if pose is None:
                print('Nessuna posa: la localizzazione è attiva? Riprova.')
                continue
            age = node.amcl_age()
            status = 'mai ricevuto' if age is None else f'{age:.0f} s fa'
            print(f'Posa: x={pose[0]}  y={pose[1]}  theta={pose[2]} rad   (ultimo /amcl_pose: {status})')
            name = input('Nome (vuoto = annulla): ').strip()
            if not name:
                print('Annullato.')
                continue
            poses = load_poses(args.file)
            if name in poses and input(f'"{name}" esiste già: sovrascrivo? [s/N] ').strip().lower() != 's':
                print('Non salvato.')
                continue
            poses[name] = {'x': pose[0], 'y': pose[1], 'theta': pose[2]}
            save_poses(args.file, poses)
            print(f'Salvato "{name}" ({len(poses)} pose nel file).')
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
