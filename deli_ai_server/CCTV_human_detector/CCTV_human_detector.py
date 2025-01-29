import cv2
import requests
from ultralytics import YOLO

# ==============================
# 1. 구역(매대) 설정
# ==============================
# 해상도(640 x 480) 예시 기준, (xmin, ymin, xmax, ymax)로 설정
zone_normal = (0,   0, 213, 480)   # (x: 0~213, y: 0~480)
zone_frozen = (214, 0, 426, 480)   # (x: 214~426, y: 0~480)
zone_fresh  = (427, 0, 640, 480)   # (x: 427~640, y: 0~480)

# ==============================
# 2. 구역 내부 판별 함수
# ==============================
def is_inside_zone(bbox, zone):
    """
    bbox: (x1, y1, x2, y2) - 객체(사람)의 bounding box 좌표
    zone: (xmin, ymin, xmax, ymax) - 구역(매대)의 사각형 영역
    """
    x1, y1, x2, y2 = bbox
    person_center = ((x1 + x2) / 2, (y1 + y2) / 2)  # bounding box 중심점
    z_xmin, z_ymin, z_xmax, z_ymax = zone

    if (z_xmin <= person_center[0] <= z_xmax) and (z_ymin <= person_center[1] <= z_ymax):
        return True
    return False

# ==============================
# 3. 구역 시각화 함수 (반투명 사각형 + 레이블)
# ==============================
def visualize_zones(frame):
    """
    화면(frame)에 3개의 구역을 반투명하게 표시하고,
    각 구역 이름을 영어로 출력한다.
    """
    # 표시 색상 및 구역 레이블(영어)
    zones = [
        (zone_normal, (255,   0,   0), "Normal Shelf"),  # 파란색
        (zone_frozen, (  0, 255, 255), "Frozen Shelf"),  # 노란색
        (zone_fresh,  (  0,   0, 255), "Fresh Shelf"),   # 빨간색
    ]

    # overlay 이미지를 하나 복사하여, 거기에 채우기(rectangle) 한 뒤
    # addWeighted로 반투명 효과를 적용
    overlay = frame.copy()
    alpha = 0.3  # 투명도 (0.0: 완전 투명, 1.0: 불투명)

    for (xmin, ymin, xmax, ymax), color, label in zones:
        # 구역 사각형을 칠한다 (두께: -1 → 내부 채움)
        cv2.rectangle(overlay, (xmin, ymin), (xmax, ymax), color, -1)

        # 구역의 왼쪽 상단에 레이블 표시 (영어)
        cv2.putText(
            overlay,
            label,
            (xmin + 10, ymin + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,                # 폰트 크기
            (255, 255, 255),  # 글자 색(흰색)
            2                 # 두께
        )

    # 원본 프레임에 overlay를 알파 블렌딩
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

# ==============================
# 4. 메인 로직
# ==============================
def main():
    # 4-1. YOLOv8 모델 로드
    model = YOLO('yolov8n.pt')  # yolov8n, yolov8s 등 원하는 모델 사용 가능

    # 4-2. 카메라(혹은 동영상) 열기
    cap = cv2.VideoCapture(0)   # 0: 기본 웹캠
    if not cap.isOpened():
        print("카메라(또는 동영상) 열기 실패")
        return

    # 4-3. 반복문 - 프레임 단위로 처리
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ---------------------------------
        # 4-3-1. 구역 시각화(반투명)
        # ---------------------------------
        visualize_zones(frame)

        # ---------------------------------
        # 4-3-2. YOLO 추론
        # ---------------------------------
        results = model(frame)

        # 구역별 사람 존재 여부
        has_person_normal = False
        has_person_frozen = False
        has_person_fresh  = False

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])  # YOLOv8 클래스 인덱스 (0 = person)
                if cls == 0:  # person
                    x1, y1, x2, y2 = box.xyxy[0]

                    # 3개 구역 중 어디에 속하는지 확인
                    if is_inside_zone((x1, y1, x2, y2), zone_normal):
                        has_person_normal = True
                    if is_inside_zone((x1, y1, x2, y2), zone_frozen):
                        has_person_frozen = True
                    if is_inside_zone((x1, y1, x2, y2), zone_fresh):
                        has_person_fresh = True

                    # 사람 bounding box 시각화 (녹색 박스)
                    cv2.rectangle(
                        frame,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (0, 255, 0),
                        2
                    )
                    cv2.putText(
                        frame,
                        "Person",
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

        # ---------------------------------
        # 4-3-3. API 호출
        # ---------------------------------
        # 주의: 여기서는 바디에 한글 키를 그대로 사용
        url = 'http://0.0.0.0:8000/api/persononshelf'
        payload = {
            "일반 매대": has_person_normal,
            "냉동 매대": has_person_frozen,
            "신선 매대": has_person_fresh
        }

        try:
            response = requests.post(url, json=payload, timeout=1.0)
            if response.status_code == 200:
                print(response.json())  # {"status": "success", ...}
            else:
                print(f"[경고] 상태 코드: {response.status_code}, 응답: {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"[오류] API 호출 실패: {e}")

        # ---------------------------------
        # 4-3-4. 결과 출력
        # ---------------------------------
        cv2.imshow("YOLOv8 Person Detection with Zones", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC 키로 종료
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
