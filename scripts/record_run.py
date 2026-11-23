#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony
"""Record /odom and /navigation/status during a Gazebo run (odom.csv, status.jsonl)."""

import json
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class Recorder(Node):
    """Write every odometry sample and status message to files in out_dir."""

    def __init__(self, out_dir: str) -> None:
        super().__init__('run_recorder')
        self.odom = open(f'{out_dir}/odom.csv', 'w')
        self.odom.write('t,x,y\n')
        self.status = open(f'{out_dir}/status.jsonl', 'w')
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/navigation/status', self.on_status, 10)

    def on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self.odom.write(f'{stamp:.3f},{p.x:.4f},{p.y:.4f}\n')

    def on_status(self, msg: String) -> None:
        data = json.loads(msg.data)
        data['t'] = self.get_clock().now().nanoseconds * 1e-9
        self.status.write(json.dumps(data) + '\n')


def main() -> None:
    """Usage: record_run.py OUT_DIR SECONDS."""
    rclpy.init()
    node = Recorder(sys.argv[1])
    end = time.time() + float(sys.argv[2])
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.odom.close()
    node.status.close()


if __name__ == '__main__':
    main()
