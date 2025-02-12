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


class YOLOv8Detector(Node):
    def __init__(self):
        super().__init__('camera_receiver')

        # UDP receiving settings (configured via commons.udp_setups)
        self.udp_ip = server["SERVER_IP"]
        self.udp_port = server["SERVER_PORT"]

        # UDP socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)

        # Load YOLO model
        self.model = '/home/naon/deli_ws/src/deli_ai_server/delibot_obstacle_detector/best.pt'
        self.yolo = YOLO(self.model)

        # Initialize YOLO result publisher
        self.detections_publisher = self.create_publisher(
            DetectionArray, "detections", 10)

        # Timer to receive data at 10 FPS
        self.timer = self.create_timer(0.1, self.receive_frame)

        self.get_logger().info(f"Camera Receiver Initialized: {self.udp_ip}:{self.udp_port}")


    def receive_frame(self):
        try:
            # Receive data from UDP socket
            data, _ = self.sock.recvfrom(65507)     # Maximum UDP packet size
            np_data = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

            if frame is not None:
                # Perform object detection using YOLO
                results = self.yolo(frame)
                results: Results = results[0]

                # Draw bounding boxes on detected objects
                annotated_frame = results[0].plot()
                cv2.imshow("YOLO Object Detection", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("Shutting down receiver.")
                    rclpy.shutdown()

                if results.boxes:
                    hypothesis = self.parse_hypothesis(results)
                    boxes = self.parse_boxes(results)
                    # distance = self.estimate_distance(results)

                detections_msg = DetectionArray()
                for i in range(len(results.boxes)):
                    aux_msg = Detection()
                    if results.boxes:
                        aux_msg.class_id = hypothesis[i]["class_id"]
                        aux_msg.class_name = hypothesis[i]["class_name"]
                        aux_msg.confidence = hypothesis[i]["confidence"]
                        aux_msg.bbox = boxes[i]

                    detections_msg.detections.append(aux_msg)

                # publish detections
                self.get_logger().info(f"Publish msg: {detections_msg}")
                self.detections_publisher.publish(detections_msg)

        except BlockingIOError:
            # Ignore if no data is received
            pass

        except Exception as e:
            self.get_logger().error(f"Error receiving frame: {str(e)}")

    
    def parse_hypothesis(self, results: Results) -> List[Dict]:
        """
        return:

        [
            {"class_id": 0, "class_name": "BOX", "confidence": 0.92},
            {"class_id": 1, "class_name": "HUMAN", "confidence": 0.87}
        ]
        """
        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "confidence": float(box_data.conf)
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

    # def estimate_distance(self, results: Results) -> float:
    #     return

        

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
