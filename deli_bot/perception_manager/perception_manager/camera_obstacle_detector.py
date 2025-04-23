import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from ai_server_msgs.msg import Detection, DetectionArray

"""
Description:
Receive YOLOv8 detections data and check if human detected.

---
Topic Subscription:
    msg type: ai_server_msgs/DetectionArray
    topic name: /yolov8_detections_info

Topic Publisher:
    msg type: std_msgs/Bool
    topic namd: /human_info
"""

class CameraObstacleDetector(Node):
    def __init__(self):
        super().__init__("camera_obstacle_detector")

        self.subscription = self.create_subscription(
            DetectionArray, "yolov8_detections_info", self.detections_callback, 10)
        
        self.publisher = self.create_publisher(
            Bool, "human_info", 10
        )

    def detections_callback(self, msg: DetectionArray):
        human_detected = False

        for detection in msg.detections:
            if detection.class_name == "HUMAN":
                human_msg = Bool()
                human_msg.data = True
                self.get_logger().info(f"Yolov8 human detected: {human_detected.data}")
                self.publisher.publish(human_msg)
                human_detected = True
                break
        
        if not human_detected:
            human_msg = Bool()
            human_msg.data = False
            self.get_logger().info(f"Yolov8 human detected: {human_msg.data}")
            self.publisher.publish(human_msg)

    def log_detection_info(self, detection: Detection):
        self.get_logger().info(
            f"Name: {detection.class_name}, "
            f"ID: {detection.class_id}), "
            f"Confidence: {detection.confidence:.2f}, "
            f"Center: ({detection.bbox.center.position.x}, {detection.bbox.center.position.y}), "
            f"Size: ({detection.bbox.size.x}, {detection.bbox.size.y})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CameraObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Lidar Obstacle Detector stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
