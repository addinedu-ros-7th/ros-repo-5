import rclpy
from rclpy.node import Node
import socket
import cv2
import numpy as np

from commons.udp_setups import server

class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')

        # UDP 수신 설정 (파라미터로 관리)
        self.udp_ip = server["SERVER_IP"]
        self.udp_port = server["SERVER_PORT"]

        # UDP 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)  # 비동기 처리

        # 타이머로 데이터 수신 (10 FPS)
        self.timer = self.create_timer(0.1, self.receive_frame)

        self.get_logger().info(f"Camera Receiver Initialized: {self.udp_ip}:{self.udp_port}")


    def receive_frame(self):
        try:
            data, _ = self.sock.recvfrom(65507)  # 최대 UDP 패킷 크기
            np_data = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

            if frame is not None:
                cv2.imshow("Received Video Stream", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("Shutting down receiver.")
                    rclpy.shutdown()

        except BlockingIOError:
            pass  # 수신할 데이터가 없으면 무시

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
