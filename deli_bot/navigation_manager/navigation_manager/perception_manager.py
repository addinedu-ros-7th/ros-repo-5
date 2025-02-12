import rclpy
from rclpy.node import Node

from ai_server_msgs.msg import DetectionArray, Detection
from nav_interfaces.msg import ObstacleDetected


class PerceptionManager(Node):
    def __init__(self):
        super().__init__('detection_subscriber')

        # Initialize YOLO result subscription
        self.detections_sub = self.create_subscription(
            DetectionArray, 'detections', self.detections_callback, 10
        )

        self.obstables_pub = self.create_publisher(
            ObstacleDetected, 'obstacles', 10
        )

        # Last published obstacles to prevent duplicates
        self.last_detected_objects = {}
        
        self.get_logger().info("Perception manager Node Initialized.")


    def detections_callback(self, msg: DetectionArray):
        # self.get_logger().info(f"Received {len(msg.detections)} detections")

        for detection in msg.detections:
            obstacle_id = detection.class_id
            obstacle_name = detection.class_name
            distance = self.estimate_distance(detection)

            # Check if the same object was already published with similar data
            if (obstacle_id in self.last_detected_objects and
                abs(self.last_detected_objects[obstacle_id]["distance"] - distance) < 0.1):
                continue  # Skip publishing

            # Create ObstacleDetected message
            obstacle_msg = ObstacleDetected()
            obstacle_msg.id = obstacle_id
            obstacle_msg.object_type = obstacle_name
            obstacle_msg.distance = distance

            # Publish ObstacleDetected message
            self.obstacle_publisher.publish(obstacle_msg)
            self.get_logger().info(f"Published Obstacle: {obstacle_msg}")


    def estimate_distance(self, detection):
        # Simplified distance estimation (e.g., based on bounding box size)
        size_factor = (detection.bbox.size.x + detection.bbox.size.y) / 2
        distance = max(0.5, 1000 / size_factor)  # Example formula
        return distance


    def log_obstacles(self, detection: Detection):
        self.get_logger().info(
            f"Class: {detection.class_name} "
            f"(ID: {detection.class_id}), "
            f"Confidence: {detection.confidence:.2f}, "
            f"Center: ({detection.bbox.center.position.x}, {detection.bbox.center.position.y}), "
            f"Size: ({detection.bbox.size.x}, {detection.bbox.size.y})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Perception manager Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
