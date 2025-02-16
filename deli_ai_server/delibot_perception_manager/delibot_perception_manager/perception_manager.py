import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np
from typing import List, Dict
import time
from commons.udp_setups import server

from ultralytics import YOLO 
from ultralytics.engine.results import Results, Boxes

from ai_server_msgs.msg import BoundingBox2D, Detection, DetectionArray


CONFIDENCE_THRESHOLD = 0.7

class YOLOv8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')

        # UDP receiving settings (configured via commons.udp_setups)
        self.udp_ip = server["SERVER_IP"]
        self.udp_port = server["SERVER_PORT"]

        # UDP socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)

        # Load YOLO model
        self.model = '/home/naon/deli_ws/src/deli_ai_server/delibot_perception_manager/best.pt'
        self.yolo = YOLO(self.model)

        # Initialize YOLO result publisher
        self.detections_publisher = self.create_publisher(
            DetectionArray, "detections", 10)

        # Timer to receive data at 10 FPS
        self.timer = self.create_timer(0.1, self.receive_frame)

        # Human detected status
        self.human_detected = False

        self.get_logger().info(f"Camera Receiver Initialized: {self.udp_ip}:{self.udp_port}")


    def receive_frame(self):
        try:
            # Receive data from UDP socket
            data, _ = self.sock.recvfrom(65507)  # Maximum UDP packet size
            np_data = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

            if frame is not None:
                # Height threshold setups
                frame_height = frame.shape[0]  # Get image height
                stop_threshold = (frame_height) // 2  # Bottom 1/2
                caution_threshold = frame_height // 3  # Middle 1/3

                status_text = "DRIVING"
                color = (0, 255, 0)

                # Perform object detection using YOLO
                results = self.yolo(frame)
                if len(results) > 0 and results[0].boxes is not None:
                    results: Results = results[0]

                    for box_data in results.boxes:
                        confidence = float(box_data.conf)
                        class_id = int(box_data.cls)
                        class_name = self.yolo.names[class_id]

                        if confidence < CONFIDENCE_THRESHOLD:
                            continue

                        bbox = box_data.xywh[0]
                        x_min = int(bbox[0] - bbox[2] / 2)
                        x_max = int(bbox[0] + bbox[2] / 2)
                        y_min = int(bbox[1] - bbox[3] / 2)
                        y_max = int(bbox[1] + bbox[3] / 2)
                        label = f"{class_name}: {confidence:.2f}"

                        if class_name == "HUMAN":
                            self.human_detected = True

                            if y_max >= stop_threshold:  # Person in the bottom 1/3
                                status_text = "STOP!"
                                color = (0, 0, 255)  # Red
                            elif y_max >= caution_threshold:  # Person in the middle 1/3
                                status_text = "CAUTION: PEDESTRIAN AHEAD"
                                color = (0, 165, 255)  # Orange
                            else:   # Person in the top 1/3
                                status_text = "DRIVING..."
                                color = (0, 255, 0)  # Green

                        # Draw bounding boxes
                        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
                        cv2.putText(frame, label, (x_min, y_max + 20), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.7, color, 2, cv2.LINE_AA)

                    # Display status text
                    cv2.putText(frame, status_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                1, color, 2, cv2.LINE_AA)

                    cv2.imshow("YOLO Object Detection", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.get_logger().info("Shutting down receiver.")
                        rclpy.shutdown()

        except BlockingIOError:
            # Ignore if no data is received
            pass

        except Exception as e:
            self.get_logger().error(f"Error receiving frame: {str(e)}")

    
    def parse_hypothesis(self, results: Results) -> List[Dict]:
        """
        Extract bounding boxes for valid detections.

        return:
        [
            {"class_id": 0, "class_name": "BOX", "confidence": 0.92},
            {"class_id": 1, "class_name": "HUMAN", "confidence": 0.87}
        ]
        """
        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            confidence = float(box_data.conf)
            # Confidence filtering
            if confidence < CONFIDENCE_THRESHOLD:
                continue  # Skip low-confidence detections

            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "confidence": confidence
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list


    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:
        """
        return:

        BoundingBox2D:
            center:
                x: 320.0
                y: 240.0
            size:
                x: 100.0
                y: 150.0

        """
        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:
            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list

    def estimate_distance(self, detection):
        # Simplified distance estimation based on bounding box size
        size_factor = (detection.bbox.size.x + detection.bbox.size.y) / 2
        distance = max(0.5, 1000 / size_factor)  # Example formula
        return distance


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera Receiver Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
