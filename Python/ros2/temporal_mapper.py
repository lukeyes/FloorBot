#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import math


class TemporalMapper(Node):
    def __init__(self):
        super().__init__('temporal_mapper')

        # CONFIGURATION
        self.width = 300
        self.height = 300
        self.res = 0.05
        self.origin_x = -7.5
        self.origin_y = -7.5

        self.HORIZONS = {
            'default': 60000,  # 1 min
            'chair': 300000,  # 5 mins
            'person': 5000,  # 5 secs
            'couch': 14400000  # 4 hours
        }
        self.current_semantic_context = 'default'

        # MEMORY (Initialized to 0.5 Unknown)
        self.grid_state = np.full((self.height, self.width), 0.5)
        self.grid_last_seen = np.zeros((self.height, self.width))
        self.grid_horizon = np.full((self.height, self.width), self.HORIZONS['default'])

        # ROS IO
        self.pub_map = self.create_publisher(OccupancyGrid, '/temporal_map', 10)

        # QoS for LaserScan (Best Effort is safer for sensor data)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.create_subscription(String, '/detected_objects', self.semantic_callback, 10)

        self.create_timer(1.0, self.update_loop)

    def semantic_callback(self, msg):
        objects = msg.data.split(',')
        if 'person' in objects:
            self.current_semantic_context = 'person'
        elif 'chair' in objects:
            self.current_semantic_context = 'chair'
        else:
            self.current_semantic_context = 'default'

    def scan_callback(self, msg):
        now_ms = self.get_clock().now().nanoseconds / 1e6
        angle = msg.angle_min

        for r in msg.ranges:
            # Check for valid range (handling infinity/NaN from realsense)
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            cx = r * math.cos(angle)
            cy = r * math.sin(angle)
            gx = int((cx - self.origin_x) / self.res)
            gy = int((cy - self.origin_y) / self.res)

            if 0 <= gx < self.width and 0 <= gy < self.height:
                self.grid_state[gy, gx] = 1.0
                self.grid_last_seen[gy, gx] = now_ms
                self.grid_horizon[gy, gx] = self.HORIZONS.get(
                    self.current_semantic_context, self.HORIZONS['default'])

            angle += msg.angle_increment

    def update_loop(self):
        now_ms = self.get_clock().now().nanoseconds / 1e6
        ages = now_ms - self.grid_last_seen

        # Vectorized Confidence Calculation
        confidences = 1.0 - (ages / self.grid_horizon)
        confidences = np.clip(confidences, 0.0, 1.0)

        # Drift-to-Mean Math
        probs = (self.grid_state * confidences) + (0.5 * (1.0 - confidences))

        # Create Message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.info.resolution = self.res
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.data = (probs * 100).astype(np.int8).flatten().tolist()

        self.pub_map.publish(msg)


def main():
    rclpy.init()
    node = TemporalMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()