#!/usr/bin/env python3
import socket
import cv2
import numpy as np
from ultralytics import YOLO

def main():
    PC_IP = "0.0.0.0"   # 모든 인터페이스
    PC_PORT = 50000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((PC_IP, PC_PORT))
    print(f"Listening on UDP {PC_IP}:{PC_PORT}")

    # YOLO 로컬 모델 불러오기
    model_path = "/home/hunter/best.pt"
    model = YOLO(model_path)

    while True:
        data, addr = sock.recvfrom(65535)
        if not data:
            continue

        # JPEG 디코딩
        npdata = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)
        if frame is None:
            print("Failed to decode image.")
            continue

        # YOLO 추론
        results = model.predict(frame)
        annotated = results[0].plot()

        cv2.imshow("YOLO Detection", annotated)
        if cv2.waitKey(1) == 27:  # ESC
            break

    sock.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

