import rclpy
import cv2
import socket
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from picamera2 import Picamera2

from commons.udp_setups import server

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')

        # Picamera2 초기화
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration({"size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

        # UDP 설정
        self.udp_ip = server["SERVER_IP"]
        self.udp_port = server["SERVER_PORT"]
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # JPEG 압축 품질 설정
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

        # 타이머 설정 (0.1초 간격, 약 10 FPS)
        self.timer = self.create_timer(0.1, self.capture_and_send)

        self.get_logger().info("Camera Streamer Node Initialized.")
    def capture_and_send(self):
        try:
            # Picamera2로 프레임 캡처 (RGB 포맷)
            frame = self.picam2.capture_array()

            # 상하 반전 및 RGB → BGR 변환
            frame = cv2.flip(frame, 0)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # JPEG 압축
            ret, encimg = cv2.imencode('.jpg', frame, self.encode_param)
            if not ret:
                self.get_logger().error("Failed to encode image.")
                return

            data = encimg.tobytes()

            # UDP 패킷 최대 크기 확인
            if len(data) > 65507:
                self.get_logger().warn("Frame too large for UDP packet.")
                return

            # UDP 전송
            self.sock.sendto(data, (self.udp_ip, self.udp_port))
            self.get_logger().info(f"Frame Sent: {len(data)} bytes")

        except Exception as e:
            self.get_logger().error(f"Error in capture_and_send: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera Streamer Stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
