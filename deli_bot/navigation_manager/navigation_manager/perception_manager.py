import rclpy
from rclpy.node import Node
from ai_server_msgs.msg import DetectionArray, Detection


class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')

        # Initialize YOLO result subscription
        self.subscription = self.create_subscription(
            DetectionArray, 'detections', self.detections_callback, 10
        )
        self.get_logger().info("Detection Subscriber Node Initialized.")


    def detections_callback(self, msg: DetectionArray):
        self.get_logger().info(f"Received {len(msg.detections)} detections")

        for detection in msg.detections:
            self.log_detection(detection)


    def log_detection(self, detection: Detection):
        self.get_logger().info(
            f"Class: {detection.class_name} "
            f"(ID: {detection.class_id}), "
            f"Confidence: {detection.confidence:.2f}, "
            f"Center: ({detection.bbox.center.position.x}, {detection.bbox.center.position.y}), "
            f"Size: ({detection.bbox.size.x}, {detection.bbox.size.y})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Detection Subscriber Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
