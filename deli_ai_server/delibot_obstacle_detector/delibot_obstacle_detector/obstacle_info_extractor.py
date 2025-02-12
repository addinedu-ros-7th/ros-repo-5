import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np
from typing import List, Dict

from commons.udp_setups import server

from ultralytics import YOLO 
from ultralytics.engine.results import Results, Boxes

from ai_server_msgs.msg import BoundingBox2D, Detection, DetectionArray


class ObstacleInfoExtractor(Node):
    def __init__(self):
        super().__init__('obstacle_info_extractor')

        self.subscription = self.create_subscription(
            DetectionArray,'detections', self.detections_callback, 10
        )
        self.publisher = self.create_publisher(
            ObstacleInfo, 'obstacle_info', 10)
        self.get_logger().info("Obstacle Info Extractor Initialized.")

    def detections_callback(self, msg):
        center_x = (msg.x_min + msg.x_max) // 2
        center_y = (msg.y_min + msg.y_max) // 2
        width = msg.x_max - msg.x_min
        height = msg.y_max - msg.y_min

        # 간단한 거리 추정 (박스 크기에 기반)
        distance = 1000 / height if height > 0 else 0

        obstacle_msg = ObstacleInfo()
        obstacle_msg.class_name = msg.class_name
        obstacle_msg.confidence = msg.confidence
        obstacle_msg.center_x = center_x
        obstacle_msg.center_y = center_y
        obstacle_msg.width = width
        obstacle_msg.height = height
        obstacle_msg.distance = distance

        self.publisher.publish(obstacle_msg)
        self.get_logger().info(f"Published Obstacle Info: {obstacle_msg.class_name}, Distance: {distance:.2f}cm")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleInfoExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Obstacle Info Extractor Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
