#!/usr/bin/env python3
"""Set the message(s) that deliver_message will say, on the latched topic /argo/messages.

In Argo v2 the mission BT writes this topic; for tests use this tool:
    python3 tools/set_message.py vito_room "la riunione è alle 15"
    python3 tools/set_message.py vito_room "la riunione è alle 15" max_room "il pacco è arrivato"
The tool keeps publishing for a few seconds and then exits; the executor keeps the last message.
"""
import json
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    args = sys.argv[1:]
    if not args or len(args) % 2:
        print(__doc__)
        return 2
    messages = dict(zip(args[0::2], args[1::2]))
    rclpy.init()
    node = rclpy.create_node('set_message')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(String, '/argo/messages', qos)
    msg = String(data=json.dumps(messages, ensure_ascii=False))
    end = time.time() + 4.0
    while time.time() < end:   # transient local: subscribers that join meanwhile get it too
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=1.0)
    print(f'Messaggi pubblicati su /argo/messages: {msg.data}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
