import sys
import requests
from datetime import datetime
from collections import defaultdict

from PyQt5.QtWidgets import QApplication, QMainWindow, QTableView
from PyQt5 import uic
from PyQt5.QtGui import QPixmap, QStandardItemModel, QStandardItem
from PyQt5.QtCore import QTimer

# admin.ui에서 자동으로 생성되는 UI 클래스
LoginUIClass = uic.loadUiType("admin.ui")[0]

class AdminWindow(QMainWindow, LoginUIClass):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Admin")

        # --- 로봇 이미지 설정 (기존과 동일) ---
        self.robot_image1.setPixmap(QPixmap("./img/robot1.png"))
        self.robot_image2.setPixmap(QPixmap("./img/robot1.png"))
        self.robot_image3.setPixmap(QPixmap("./img/robot1.png"))
        self.robot_image4.setPixmap(QPixmap("./img/robot2.png"))
        self.robot_image5.setPixmap(QPixmap("./img/robot1.png"))
        self.robot_image6.setPixmap(QPixmap("./img/robot1.png"))
        self.robot_image7.setPixmap(QPixmap("./img/robot1.png"))

        for label in [
            self.robot_image1, self.robot_image2, self.robot_image3,
            self.robot_image4, self.robot_image5, self.robot_image6,
            self.robot_image7
        ]:
            label.setScaledContents(True)
        
        # --- 맵 설정
        self.map.setPixmap(QPixmap("./img/map.png"))




        # --- 타이머 설정 (로봇 관련만 1초마다) ---
        self.timer = QTimer(self)
        self.timer.setInterval(1000)  # 1초(1000ms)
        # 주문 테이블 갱신은 타이머와 무관 (최초 1회만)
        self.timer.timeout.connect(self.update_robot_states)
        self.timer.timeout.connect(self.update_robot_locations)
        self.timer.start()

        # 프로그램 시작 시, 최초 1회만 주문 정보를 가져와 테이블 완성 & 통계 라벨 세팅
        self.init_order_data()
        self.update_robot_logs()
        self.update_users_table()

        self.btn_update_time.clicked.connect(self.send_waiting_time_update)

    def send_waiting_time_update(self):
        """
        사용자가 선택한 매대와 입력한 시간을 API에 전송
        """
        # 선택된 매대 확인
        normal_time = int(self.lineEdit.text()) * 60 if self.radio_normal.isChecked() else 0
        cold_time = int(self.lineEdit.text()) * 60 if self.radio_cold.isChecked() else 0
        fresh_time = int(self.lineEdit.text()) * 60 if self.radio_fresh.isChecked() else 0

        # API 요청 데이터
        data = {
            "일반 매대": normal_time,
            "냉동 매대": cold_time,
            "신선 매대": fresh_time
        }

        try:
            response = requests.post("http://0.0.0.0:8000/api/robots/waiting-time", json=data)
            if response.status_code == 200:
                print("대기 시간 업데이트 성공:", response.json())
            else:
                print(f"API 요청 실패: HTTP {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"API 요청 중 오류 발생: {e}")

    def update_users_table(self):
        """
        /api/users API를 통해 사용자 목록을 받아 table3에 표시한다.
        컬럼: 이름, 아이디, 주소, 이메일
        """
        url = "http://0.0.0.0:8000/api/users"
        try:
            response = requests.get(url)
            response.raise_for_status()  # 200이 아닌 경우 예외 발생
            data = response.json()

            users = data.get("users", [])  # "users" 리스트 추출
            # 테이블에 넣을 모델 생성 (행 개수 = users 길이, 열 개수 = 4)
            model = QStandardItemModel(len(users), 4, self)
            model.setHorizontalHeaderLabels([
                "이름",
                "아이디",
                "주소",
                "이메일"
            ])

            for row, user in enumerate(users):
                name = user.get("name", "")
                user_id = user.get("id", "")
                address = user.get("address", "")
                email = user.get("email", "")

                # 각 셀에 데이터를 채움
                model.setItem(row, 0, QStandardItem(str(name)))
                model.setItem(row, 1, QStandardItem(str(user_id)))
                model.setItem(row, 2, QStandardItem(str(address)))
                model.setItem(row, 3, QStandardItem(str(email)))

            # 완성된 모델을 table3에 세팅
            self.table3.setModel(model)

        except requests.exceptions.RequestException as e:
            print(f"사용자 목록 조회 실패: {e}")

    def update_robot_logs(self):
        """
        /api/robots/logs API에서 로그를 받아와 table2에 표시.
        로봇 ID가 1,2,3번이면 '주행 로봇', 10,11,12번이면 '로봇팔'로 표시
        """
        url = "http://0.0.0.0:8000/api/robots/logs"
        try:
            response = requests.get(url)
            response.raise_for_status()  # HTTP 에러 시 예외 발생
            data = response.json()

            logs = data.get("logs", [])  # "logs" 리스트
            # 테이블에 넣을 모델 생성 (행 개수 = logs 길이, 열 개수 = 5)
            model = QStandardItemModel(len(logs), 5, self)
            model.setHorizontalHeaderLabels([
                "로봇 ID",     # 예: 1, 2, 3, 10, 11, 12
                "로봇유형",    # 주행 로봇 / 로봇팔
                "상태(이벤트)",
                "위치",
                "시간(timestamp)"
            ])

            for row, log_entry in enumerate(logs):
                robot_id = log_entry.get("robotId", "")
                status = log_entry.get("status", "")
                location = log_entry.get("location", "")
                timestamp = log_entry.get("timestamp", "")

                # 로봇 ID가 1,2,3 => 주행 로봇 / 10,11,12 => 로봇팔
                robot_type = self._get_robot_type(robot_id)

                # 표에 각 셀 데이터 삽입
                model.setItem(row, 0, QStandardItem(str(robot_id)))
                model.setItem(row, 1, QStandardItem(robot_type))
                model.setItem(row, 2, QStandardItem(status))
                model.setItem(row, 3, QStandardItem(location))
                model.setItem(row, 4, QStandardItem(timestamp))

            # 완성된 모델을 table2에 세팅
            self.table2.setModel(model)

        except requests.exceptions.RequestException as e:
            print(f"로봇 로그 조회 실패: {e}")

    def _get_robot_type(self, robot_id):
        """
        로봇 ID(문자열)를 보고 '주행 로봇' or '로봇팔' 판별 후 반환
        (1,2,3 => 주행 로봇 / 10,11,12 => 로봇팔 / 그 외 => 기타)
        """
        # robot_id가 문자열이므로, 정수로 변환 후 비교(예외 처리)
        try:
            rid = int(robot_id)
        except ValueError:
            # 숫자가 아니면 기타
            return "기타"

        if rid in [1, 2, 3]:
            return "주행 로봇"
        elif rid in [10, 11, 12]:
            return "로봇팔"
        else:
            return "기타"

    def init_order_data(self):
        """
        프로그램 시작 시(또는 필요한 시점에) 한 번만 호출하여
        /api/orders 데이터를 가져와 테이블1에 세팅하고,
        label1/label2/label3/label4에 필요한 통계 정보를 표시한다.
        """
        try:
            response = requests.get("http://0.0.0.0:8000/api/orders")
            if response.status_code == 200:
                data = response.json()
                orders = data.get("orders", [])

                # (1) 테이블 채우기
                self.fill_order_table(orders)

                # (2) 통계 정보 계산 후 라벨에 표시
                self.update_order_stats(orders)
            else:
                print(f"주문 목록 조회 에러: HTTP {response.status_code}")
        except Exception as e:
            print(f"주문 목록 조회 실패: {e}")

    def fill_order_table(self, orders):
        """
        주문 목록을 받아 table1(QTableView)에 표시
        (요구 컬럼: 주문번호, 주문상태, 일반1, 일반2, 신선1, 신선2, 냉동1, 냉동2, 가격, 시작 시간, 완료 시간)
        """
        model = QStandardItemModel(len(orders), 11, self)
        model.setHorizontalHeaderLabels([
            "주문번호",
            "주문상태",
            "일반1",
            "일반2",
            "신선1",
            "신선2",
            "냉동1",
            "냉동2",
            "가격",
            "시작 시간",
            "완료 시간"
        ])

        for row, order in enumerate(orders):
            order_id = order.get("orderId", "")
            status = order.get("status", "")
            items = order.get("items", {})
            # 아이템 별 개수 꺼내기 (없으면 0)
            normal1 = items.get("일반1", 0)
            normal2 = items.get("일반2", 0)
            fresh1 = items.get("신선1", 0)
            fresh2 = items.get("신선2", 0)
            frozen1 = items.get("냉동1", 0)
            frozen2 = items.get("냉동2", 0)
            price = order.get("price", 0)
            created_at = order.get("createdAt", "")
            over_at = order.get("overAt", "")

            model.setItem(row, 0, QStandardItem(str(order_id)))
            model.setItem(row, 1, QStandardItem(str(status)))
            model.setItem(row, 2, QStandardItem(str(normal1)))
            model.setItem(row, 3, QStandardItem(str(normal2)))
            model.setItem(row, 4, QStandardItem(str(fresh1)))
            model.setItem(row, 5, QStandardItem(str(fresh2)))
            model.setItem(row, 6, QStandardItem(str(frozen1)))
            model.setItem(row, 7, QStandardItem(str(frozen2)))
            model.setItem(row, 8, QStandardItem(str(price)))
            model.setItem(row, 9, QStandardItem(str(created_at)))
            model.setItem(row, 10, QStandardItem(str(over_at)))

        self.table1.setModel(model)

    def update_order_stats(self, orders):
        """
        주문 목록을 기반으로 통계 계산 후,
        label1, label2, label3, label4에 각각 표시.
        - label1: 평균 작업 완료 시간 (초 단위 또는 분/초)
        - label2: 현재 대기중인 주문 갯수 (completed가 아닌 주문)
        - label3: 가장 많이 팔린 물품명
        - label4: 총 매출 (completed인 주문의 price 합)
        """
        # (a) 평균 작업 완료 시간 (completed인 주문 중 createdAt ~ overAt 평균)
        time_diffs = []
        for order in orders:
            if order.get("status") == "completed":
                created_str = order.get("createdAt")
                over_str = order.get("overAt")
                if created_str and over_str:  # overAt이 null이 아닌 경우만
                    try:
                        dt_created = datetime.fromisoformat(created_str)
                        dt_over = datetime.fromisoformat(over_str)
                        diff_sec = (dt_over - dt_created).total_seconds()
                        time_diffs.append(diff_sec)
                    except ValueError:
                        # datetime 파싱 실패 시 무시
                        pass

        if len(time_diffs) > 0:
            avg_sec = sum(time_diffs) / len(time_diffs)
            # 분/초로 변환 예시 (원하시면 형식 자유롭게)
            avg_min = int(avg_sec // 60)
            remainder_sec = int(avg_sec % 60)
            self.label1.setText(f"평균 작업 완료 시간: {avg_min}분 {remainder_sec}초")
        else:
            self.label1.setText("평균 작업 완료 시간: N/A")

        # (b) 현재 대기중인 주문 갯수 (completed가 아닌 모든 주문)
        waiting_count = sum(1 for o in orders if o.get("status") != "completed")
        self.label2.setText(f"현재 대기중인 주문: {waiting_count}건")

        # (c) 가장 많이 팔린 물품명 (모든 주문의 items 기준)
        item_counter = defaultdict(int)
        for order in orders:
            items = order.get("items", {})
            for item_name, qty in items.items():
                item_counter[item_name] += qty

        if item_counter:
            top_item = max(item_counter, key=item_counter.get)  # 가장 많이 팔린 key
            self.label3.setText(f"가장 많이 팔린 물품: {top_item}")
        else:
            self.label3.setText("가장 많이 팔린 물품: 없음")

        # (d) 총 매출 (completed 주문의 price 합)
        total_revenue = sum(o.get("price", 0) for o in orders if o.get("status") == "completed")
        self.label4.setText(f"총 매출: {total_revenue}원")

    # ---------------------------
    # 이하 로봇 상태/위치 함수는 기존과 동일
    # ---------------------------
    def update_robot_states(self):
        """
        1초마다 로봇 상태 (/api/robots/status) 갱신
        """
        try:
            response = requests.get("http://0.0.0.0:8000/api/robots/status")
            if response.status_code == 200:
                data = response.json()
                robots = data.get("robots", [])

                for i in range(4):
                    if i >= len(robots):
                        self._set_robot_state_text(i+1, "로봇 정보 없음")
                    else:
                        robot_info = robots[i]
                        text = (
                            f"Robot ID  : {robot_info.get('robotId', '')}\n"
                            f"Name      : {robot_info.get('name', '')}\n"
                            f"Type      : {robot_info.get('type', '')}\n"
                            f"Battery   : {robot_info.get('batteryLevel', '')}\n"
                            f"Status    : {robot_info.get('status', '')}\n"
                            f"Wait Time : {robot_info.get('waitingTime', '')}\n"
                            f"Location  : x={robot_info.get('location', {}).get('x', '')}, "
                            f"y={robot_info.get('location', {}).get('y', '')}\n"
                            f"Remain : {robot_info.get('remainingItems',{})}\n"
                            f"Carrying : {robot_info.get('carryingItems',{})}\n"
                        )   
                        self._set_robot_state_text(i+1, text)
            else:
                error_text = f"상태 조회 에러: HTTP {response.status_code}"
                self._set_all_robot_state_text(error_text)
        except Exception as e:
            self._set_all_robot_state_text(f"상태 조회 실패: {str(e)}")

    def update_robot_locations(self):
        """
        1초마다 /api/robots/locations 로봇 위치 갱신
        """
        try:
            response = requests.get("http://0.0.0.0:8000/api/robots/locations")
            if response.status_code == 200:
                data = response.json()
                locations = data.get("locations", [])
                robot_label_map = {
                    "1": self.robot_image5,
                    "2": self.robot_image6,
                    "3": self.robot_image7,
                }
                for loc in locations:
                    robot_id = str(loc.get("robotId", ""))
                    x = loc.get("x", 0.0)
                    y = loc.get("y", 0.0)
                    if robot_id in robot_label_map:
                        label = robot_label_map[robot_id]
                        px, py = self._map_to_widget_coords(x, y)
                        label.move(px, py)
            else:
                print(f"위치 조회 에러: HTTP {response.status_code}")
        except Exception as e:
            print(f"위치 조회 실패: {e}")

    def _map_to_widget_coords(self, x, y):
        """
        좌표 변환 (단순 스케일 예시)
        """
        scale = 1000
        px = int(x * scale) + (- 75 + 70)
        py = int(y * scale) + (- 75 + 70) 
        print(px, ",",py)
        return px, py

    def _set_robot_state_text(self, robot_index, text):
        if robot_index == 1:
            self.robot_state1.setText(text)
        elif robot_index == 2:
            self.robot_state2.setText(text)
        elif robot_index == 3:
            self.robot_state3.setText(text)
        elif robot_index == 4:
            self.robot_state4.setText(text)

    def _set_all_robot_state_text(self, text):
        self.robot_state1.setText(text)
        self.robot_state2.setText(text)
        self.robot_state3.setText(text)
        self.robot_state4.setText(text)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    admin_window = AdminWindow()
    admin_window.show()
    sys.exit(app.exec_())
