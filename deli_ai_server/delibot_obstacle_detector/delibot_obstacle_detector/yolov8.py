import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np

from commons.udp_setups import server

from ultralytics import YOLO 
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints




class CameraReceiver(Node):
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
        self.model = YOLO('/home/naon/deli_ws/src/deli_ai_server/delibot_obstacle_detector/best.pt')

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
                results = self.model(frame)
                # Draw bounding boxes on detected objects
                annotated_frame = results[0].plot()

                cv2.imshow("YOLO Object Detection", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("Shutting down receiver.")
                    rclpy.shutdown()

        except BlockingIOError:
            # Ignore if no data is received
            pass

        except Exception as e:
            self.get_logger().error(f"Error receiving frame: {str(e)}")



def main(args=None):
    rclpy.init(args=args)
    node = CameraReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera Receiver Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
