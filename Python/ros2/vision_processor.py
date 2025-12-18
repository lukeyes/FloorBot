#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')

        # Load model (auto-downloads to ~/.config/Ultralytics)
        self.get_logger().info("Loading YOLOv8 Nano model...")
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        # Subscribe to D435 RGB
        # Reliability 'Best Effort' is often needed for RealSense over DDS
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Updated topic standard for RS wrapper
            self.image_callback,
            qos_profile)

        self.pub = self.create_publisher(String, '/detected_objects', 10)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(frame, verbose=False)

            detected = []
            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    name = self.model.names[cls_id]
                    detected.append(name)

            if detected:
                msg_out = String()
                msg_out.data = ",".join(set(detected))
                self.pub.publish(msg_out)
        except Exception as e:
            self.get_logger().warn(f"Vision processing failed: {e}")


def main():
    rclpy.init()
    node = VisionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()