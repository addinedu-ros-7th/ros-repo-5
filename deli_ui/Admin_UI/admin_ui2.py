import sys
import requests
from datetime import datetime
from collections import defaultdict

from PyQt5.QtWidgets import QApplication, QMainWindow, QTableView, QLabel
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

        # ------------------------------------------------------------
        # 기존 로봇 아이콘 (직접 맵 위에 띄워놓는 아이콘) 설정 예시
        # ------------------------------------------------------------
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
        
        # ------------------------------------------------------------
        # 기존 맵 (map.png) 표시
        # ------------------------------------------------------------
        self.map.setPixmap(QPixmap("./img/map.png"))

        # ------------------------------------------------------------
        # 추가된 map_2 ~ map_43 QLabel에 대해, 대응되는 이미지 파일을 설정하고 초기에는 모두 hide
        # (아래는 예시 매핑이므로, 실제 ui 파일의 라벨 이름/순서에 맞게 수정 필요)
        # ------------------------------------------------------------
        # 각각의 QLabel을 ui에서 findChild 하거나, 디자이너에서 "objectName"을 정확히 맞춘 뒤
        # self.map_2, self.map_3, ... 처럼 바로 사용합니다.
        self.map_2.setPixmap(QPixmap("./img/1시작점.png"))
        self.map_3.setPixmap(QPixmap("./img/1일반매대.png"))
        self.map_4.setPixmap(QPixmap("./img/1냉동매대.png"))
        self.map_5.setPixmap(QPixmap("./img/1신선매대.png"))
        self.map_6.setPixmap(QPixmap("./img/1시작점-일반매대.png"))
        self.map_7.setPixmap(QPixmap("./img/1시작점-냉동매대.png"))
        self.map_8.setPixmap(QPixmap("./img/1시작점-신선매대.png"))
        self.map_9.setPixmap(QPixmap("./img/1일반매대-냉동매대.png"))
        self.map_10.setPixmap(QPixmap("./img/1일반매대-신선매대.png"))
        self.map_11.setPixmap(QPixmap("./img/1신선매대-냉동매대.png"))
        self.map_12.setPixmap(QPixmap("./img/1하역장-시작점.png"))
        self.map_13.setPixmap(QPixmap("./img/1하역장-일반매대.png"))
        self.map_14.setPixmap(QPixmap("./img/1하역장-냉동매대.png"))
        self.map_15.setPixmap(QPixmap("./img/1하역장-신선매대.png"))

        self.map_16.setPixmap(QPixmap("./img/2시작점.png"))
        self.map_17.setPixmap(QPixmap("./img/2일반매대.png"))
        self.map_18.setPixmap(QPixmap("./img/2냉동매대.png"))
        self.map_19.setPixmap(QPixmap("./img/2신선매대.png"))
        self.map_20.setPixmap(QPixmap("./img/2시작점-일반매대.png"))
        self.map_21.setPixmap(QPixmap("./img/2시작점-냉동매대.png"))
        self.map_22.setPixmap(QPixmap("./img/2시작점-신선매대.png"))
        self.map_23.setPixmap(QPixmap("./img/2일반매대-냉동매대.png"))
        self.map_24.setPixmap(QPixmap("./img/2일반매대-신선매대.png"))
        self.map_25.setPixmap(QPixmap("./img/2신선매대-냉동매대.png"))
        self.map_26.setPixmap(QPixmap("./img/2하역장-시작점.png"))
        self.map_27.setPixmap(QPixmap("./img/2하역장-일반매대.png"))
        self.map_28.setPixmap(QPixmap("./img/2하역장-냉동매대.png"))
        self.map_29.setPixmap(QPixmap("./img/2하역장-신선매대.png"))

        self.map_30.setPixmap(QPixmap("./img/3시작점.png"))
        self.map_31.setPixmap(QPixmap("./img/3일반매대.png"))
        self.map_32.setPixmap(QPixmap("./img/3냉동매대.png"))
        self.map_33.setPixmap(QPixmap("./img/3신선매대.png"))
        self.map_34.setPixmap(QPixmap("./img/3시작점-일반매대.png"))
        self.map_35.setPixmap(QPixmap("./img/3시작점-냉동매대.png"))
        self.map_36.setPixmap(QPixmap("./img/3시작점-신선매대.png"))
        self.map_37.setPixmap(QPixmap("./img/3일반매대-냉동매대.png"))
        self.map_38.setPixmap(QPixmap("./img/3일반매대-신선매대.png"))
        self.map_39.setPixmap(QPixmap("./img/3신선매대-냉동매대.png"))
        self.map_40.setPixmap(QPixmap("./img/3하역장-시작점.png"))
        self.map_41.setPixmap(QPixmap("./img/3하역장-일반매대.png"))
        self.map_42.setPixmap(QPixmap("./img/3하역장-냉동매대.png"))
        self.map_43.setPixmap(QPixmap("./img/3하역장-신선매대.png"))
        

        self.label_7.setPixmap(QPixmap("./img/Rectangle1.png"))
        self.label_6.setPixmap(QPixmap("./img/Rectangle2.png"))
        self.label_5.setPixmap(QPixmap("./img/Rectangle3.png"))

        # 일괄 hide
        for i in range(2, 44):
            lbl = self.findChild(QLabel, f"map_{i}")
            if lbl:
                lbl.hide()

        # ------------------------------------------------------------
        # 이미지 표시를 쉽게 하기 위해, "로봇ID + 스테이션(혹은 이동경로)" => QLabel 매핑을 dict로 구성
        # ------------------------------------------------------------
        # 키 예시: "1시작점", "1일반매대", "1시작점-일반매대", ...
        self.station_label_map = {
            # Robot1
            "1시작점": self.map_2,
            "1일반매대": self.map_3,
            "1냉동매대": self.map_4,
            "1신선매대": self.map_5,
            "1시작점-일반매대": self.map_6,
            "1일반매대-시작점": self.map_6,  # 동일 이미지
            "1시작점-냉동매대": self.map_7,
            "1냉동매대-시작점": self.map_7,  # 동일 이미지
            "1시작점-신선매대": self.map_8,
            "1신선매대-시작점": self.map_8,  # 동일 이미지
            "1일반매대-냉동매대": self.map_9,
            "1냉동매대-일반매대": self.map_9,
            "1일반매대-신선매대": self.map_10,
            "1신선매대-일반매대": self.map_10,
            "1신선매대-냉동매대": self.map_11,
            "1냉동매대-신선매대": self.map_11,
            "1하역장-시작점": self.map_12,
            "1시작점-하역장": self.map_12,  # 동일 이미지
            "1하역장-일반매대": self.map_13,
            "1일반매대-하역장": self.map_13,
            "1하역장-냉동매대": self.map_14,
            "1냉동매대-하역장": self.map_14,
            "1하역장-신선매대": self.map_15,
            "1신선매대-하역장": self.map_15,

            # Robot2
            "2시작점": self.map_16,
            "2일반매대": self.map_17,
            "2냉동매대": self.map_18,
            "2신선매대": self.map_19,
            "2시작점-일반매대": self.map_20,
            "2일반매대-시작점": self.map_20,
            "2시작점-냉동매대": self.map_21,
            "2냉동매대-시작점": self.map_21,
            "2시작점-신선매대": self.map_22,
            "2신선매대-시작점": self.map_22,
            "2일반매대-냉동매대": self.map_23,
            "2냉동매대-일반매대": self.map_23,
            "2일반매대-신선매대": self.map_24,
            "2신선매대-일반매대": self.map_24,
            "2신선매대-냉동매대": self.map_25,
            "2냉동매대-신선매대": self.map_25,
            "2하역장-시작점": self.map_26,
            "2시작점-하역장": self.map_26,
            "2하역장-일반매대": self.map_27,
            "2일반매대-하역장": self.map_27,
            "2하역장-냉동매대": self.map_28,
            "2냉동매대-하역장": self.map_28,
            "2하역장-신선매대": self.map_29,
            "2신선매대-하역장": self.map_29,

            # Robot3
            "3시작점": self.map_30,
            "3일반매대": self.map_31,
            "3냉동매대": self.map_32,
            "3신선매대": self.map_33,
            "3시작점-일반매대": self.map_34,
            "3일반매대-시작점": self.map_34,
            "3시작점-냉동매대": self.map_35,
            "3냉동매대-시작점": self.map_35,
            "3시작점-신선매대": self.map_36,
            "3신선매대-시작점": self.map_36,
            "3일반매대-냉동매대": self.map_37,
            "3냉동매대-일반매대": self.map_37,
            "3일반매대-신선매대": self.map_38,
            "3신선매대-일반매대": self.map_38,
            "3신선매대-냉동매대": self.map_39,
            "3냉동매대-신선매대": self.map_39,
            "3하역장-시작점": self.map_40,
            "3시작점-하역장": self.map_40,
            "3하역장-일반매대": self.map_41,
            "3일반매대-하역장": self.map_41,
            "3하역장-냉동매대": self.map_42,
            "3냉동매대-하역장": self.map_42,
            "3하역장-신선매대": self.map_43,
            "3신선매대-하역장": self.map_43,
        }

        # ------------------------------------------------------------
        # 타이머 설정 (기존과 동일)
        # ------------------------------------------------------------
        self.timer = QTimer(self)
        self.timer.setInterval(1000)  # 1초(1000ms)
        self.timer.timeout.connect(self.update_robot_states)
        self.timer.timeout.connect(self.update_robot_locations)
        self.timer.timeout.connect(self.init_order_data)
        self.timer.timeout.connect(self.update_assigned_queued_orders)
        self.timer.start()

        # 프로그램 시작 시, 최초 1회만 주문 정보를 가져와 테이블 완성 & 통계 라벨 세팅
        #self.init_order_data()
        self.update_robot_logs()
        self.update_users_table()

        # 대기시간 업데이트 버튼
        self.btn_update_time.clicked.connect(self.send_waiting_time_update)

    def update_assigned_queued_orders(self):
        """
        /api/orders에서 주문 목록을 가져와서,
        status가 assigned/queued 인 주문만 필터링하여
        Ordertable(QTableView)에 표시.
        컬럼: 주문번호, 주문상태, 가격, 시작 시간
        """
        try:
            response = requests.get("http://0.0.0.0:8000/api/orders")
            if response.status_code == 200:
                data = response.json()
                orders = data.get("orders", [])

                # 1) assigned 혹은 queued인 주문만 필터링
                filtered_orders = [o for o in orders if o.get("status") in ("assigned", "queued")]

                # 2) 테이블에 표시할 모델 구성 (컬럼 4개)
                model = QStandardItemModel(len(filtered_orders), 4, self)
                model.setHorizontalHeaderLabels([
                    "주문번호", "주문상태", "가격", "시작 시간"
                ])

                # 3) 필터링된 주문 리스트를 순회하며 행 추가
                for row, order in enumerate(filtered_orders):
                    order_id = order.get("orderId", "")
                    status = order.get("status", "")
                    price = order.get("price", 0)
                    created_at = order.get("createdAt", "")

                    model.setItem(row, 0, QStandardItem(str(order_id)))
                    model.setItem(row, 1, QStandardItem(str(status)))
                    model.setItem(row, 2, QStandardItem(str(price)))
                    model.setItem(row, 3, QStandardItem(str(created_at)))

                # 4) Ordertable(새로 추가한 QTableView)에 세팅
                self.Ordertable.setModel(model)

            else:
                print(f"주문 목록 조회 에러: HTTP {response.status_code}")
        except Exception as e:
            print(f"주문 목록 조회 실패: {e}")


    # ----------------------------------------------------------------
    # (1) 대기 시간 업데이트 (API)
    # ----------------------------------------------------------------
    def send_waiting_time_update(self):
        """
        사용자가 선택한 매대와 입력한 시간을 API에 전송
        """
        # 선택된 매대 확인
        normal_time = int(self.lineEdit.text()) * 60 if self.radio_normal.isChecked() else 0
        cold_time = int(self.lineEdit.text()) * 60 if self.radio_cold.isChecked() else 0
        fresh_time = int(self.lineEdit.text()) * 60 if self.radio_fresh.isChecked() else 0

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

    # ----------------------------------------------------------------
    # (2) 사용자 목록 테이블 (table3)
    # ----------------------------------------------------------------
    def update_users_table(self):
        """
        /api/users API를 통해 사용자 목록을 받아 table3에 표시
        컬럼: 이름, 아이디, 주소, 이메일
        """
        url = "http://0.0.0.0:8000/api/users"
        try:
            response = requests.get(url)
            response.raise_for_status()
            data = response.json()

            users = data.get("users", [])
            model = QStandardItemModel(len(users), 4, self)
            model.setHorizontalHeaderLabels(["이름", "아이디", "주소", "이메일"])

            for row, user in enumerate(users):
                name = user.get("name", "")
                user_id = user.get("id", "")
                address = user.get("address", "")
                email = user.get("email", "")

                model.setItem(row, 0, QStandardItem(str(name)))
                model.setItem(row, 1, QStandardItem(str(user_id)))
                model.setItem(row, 2, QStandardItem(str(address)))
                model.setItem(row, 3, QStandardItem(str(email)))

            self.table3.setModel(model)
        except requests.exceptions.RequestException as e:
            print(f"사용자 목록 조회 실패: {e}")

    # ----------------------------------------------------------------
    # (3) 로봇 로그 테이블 (table2)
    # ----------------------------------------------------------------
    def update_robot_logs(self):
        """
        /api/robots/logs API에서 로그를 받아와 table2에 표시
        로봇 ID가 1,2,3이면 '주행 로봇', 10,11,12면 '로봇팔'
        """
        url = "http://0.0.0.0:8000/api/robots/logs"
        try:
            response = requests.get(url)
            response.raise_for_status()
            data = response.json()

            logs = data.get("logs", [])
            model = QStandardItemModel(len(logs), 5, self)
            model.setHorizontalHeaderLabels([
                "로봇 ID", "로봇유형", "상태(이벤트)", "위치", "시간(timestamp)"
            ])

            for row, log_entry in enumerate(logs):
                robot_id = log_entry.get("robotId", "")
                status = log_entry.get("status", "")
                location = log_entry.get("location", "")
                timestamp = log_entry.get("timestamp", "")

                robot_type = self._get_robot_type(robot_id)

                model.setItem(row, 0, QStandardItem(str(robot_id)))
                model.setItem(row, 1, QStandardItem(robot_type))
                model.setItem(row, 2, QStandardItem(status))
                model.setItem(row, 3, QStandardItem(location))
                model.setItem(row, 4, QStandardItem(timestamp))

            self.table2.setModel(model)

        except requests.exceptions.RequestException as e:
            print(f"로봇 로그 조회 실패: {e}")

    def _get_robot_type(self, robot_id):
        """
        로봇 ID 정수값을 보고 '주행 로봇' or '로봇팔' 판별
        """
        try:
            rid = int(robot_id)
        except ValueError:
            return "기타"

        if rid in [1, 2, 3]:
            return "주행 로봇"
        elif rid in [10, 11, 12]:
            return "로봇팔"
        else:
            return "기타"

    # ----------------------------------------------------------------
    # (4) 주문 정보 테이블 (table1) + 통계 라벨들(label1~label4)
    # ----------------------------------------------------------------
    def init_order_data(self):
        """
        /api/orders 데이터를 가져와 테이블1에 세팅 + 통계 라벨 세팅
        """
        try:
            response = requests.get("http://0.0.0.0:8000/api/orders")
            if response.status_code == 200:
                data = response.json()
                orders = data.get("orders", [])

                # 테이블 채우기
                self.fill_order_table(orders)
                # 통계 정보 갱신
                self.update_order_stats(orders)
            else:
                print(f"주문 목록 조회 에러: HTTP {response.status_code}")
        except Exception as e:
            print(f"주문 목록 조회 실패: {e}")

    def fill_order_table(self, orders):
        """
        table1(QTableView)에 주문 목록 표시
        컬럼: 주문번호, 주문상태, 일반1, 일반2, 신선1, 신선2, 냉동1, 냉동2, 가격, 시작 시간, 완료 시간
        """
        model = QStandardItemModel(len(orders), 11, self)
        model.setHorizontalHeaderLabels([
            "주문번호", "주문상태", "일반1", "일반2", "신선1",
            "신선2", "냉동1", "냉동2", "가격", "시작 시간", "완료 시간"
        ])

        for row, order in enumerate(orders):
            order_id = order.get("orderId", "")
            status = order.get("status", "")
            items = order.get("items", {})
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
        주문 목록 기반 통계 라벨:
        - label1: 평균 작업 완료 시간
        - label2: 현재 대기중인 주문 갯수
        - label3: 가장 많이 팔린 물품
        - label4: 총 매출 (completed 주문 가격 합)
        """
        # (a) 평균 작업 완료 시간
        from_stat = []
        for order in orders:
            if order.get("status") == "completed":
                created_str = order.get("createdAt")
                over_str = order.get("overAt")
                if created_str and over_str:
                    try:
                        dt_created = datetime.fromisoformat(created_str)
                        dt_over = datetime.fromisoformat(over_str)
                        diff_sec = (dt_over - dt_created).total_seconds()
                        from_stat.append(diff_sec)
                    except ValueError:
                        pass

        if len(from_stat) > 0:
            avg_sec = sum(from_stat) / len(from_stat)
            avg_min = int(avg_sec // 60)
            remainder_sec = int(avg_sec % 60)
            self.label1.setText(f"평균 작업 완료 시간: {avg_min}분 {remainder_sec}초")
        else:
            self.label1.setText("평균 작업 완료 시간: N/A")

        # (b) 대기중인 주문 갯수
        waiting_count = sum(1 for o in orders if o.get("status") != "completed")
        self.label2.setText(f"현재 대기중인 주문: {waiting_count}건")

        # (c) 가장 많이 팔린 물품명
        from collections import defaultdict
        item_counter = defaultdict(int)
        for order in orders:
            items = order.get("items", {})
            for item_name, qty in items.items():
                item_counter[item_name] += qty

        if item_counter:
            top_item = max(item_counter, key=item_counter.get)
            self.label3.setText(f"가장 많이 팔린 물품: {top_item}")
        else:
            self.label3.setText("가장 많이 팔린 물품: 없음")

        # (d) 총 매출
        total_revenue = sum(o.get("price", 0) for o in orders if o.get("status") == "completed")
        self.label4.setText(f"총 매출: {total_revenue}원")

    # ----------------------------------------------------------------
    # (5) 로봇 상태 갱신 => 로봇 상태 텍스트, 그리고 map_x 표시
    # ----------------------------------------------------------------
    def update_robot_states(self):
        """
        1초마다 /api/robots/status 로봇 상태 갱신
        + map_2~map_43 중 해당하는 이미지를 show/hide
        """
        try:
            response = requests.get("http://0.0.0.0:8000/api/robots/status")
            if response.status_code == 200:
                data = response.json()
                robots = data.get("robots", [])

                # 1) 텍스트 박스에 로봇 상태 표시 (기존)
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

                # 2) map_x 라벨들 중에서, 현재 로봇 상태에 맞는 이미지를 show/hide
                self._update_map_images(robots)

            else:
                error_text = f"상태 조회 에러: HTTP {response.status_code}"
                self._set_all_robot_state_text(error_text)
        except Exception as e:
            self._set_all_robot_state_text(f"상태 조회 실패: {str(e)}")

    def _update_map_images(self, robots):
        """
        로봇 status를 해석하여 map_2~map_43 라벨 중 1개를 show (나머지는 hide).
        - 로봇 N(=1,2,3)이 '대기중'이면 => N + 현재 스테이션 으로 단일 이미지 표시
        - 로봇 N이 'XX에서 YY로 이동중'이면 => N + "XX-YY" 이미지 표시
        - 로봇 N이 'XX 매대에서 물건담기 작업중' => N + "XX" 단일 이미지
        etc.
        """
        # 모든 라벨 hide
        self._hide_all_map_labels()

        for robot in robots:
            r_id = str(robot.get("robotId", ""))
            status_str = robot.get("status", "")

            # station 이름 치환용 dict
            station_map = {
                "출발지": "시작점",
                "목적지": "하역장",
                "일반": "일반매대",
                "냉동": "냉동매대",
                "신선": "신선매대"
            }

            # 1) "xxx에서 대기중"
            # 2) "xxx에서 yyy로 이동중"
            # 3) "xxx 매대에서 물건담기 작업중"
            # 4) "주문처리 시작" (=> 시작점?)
            # ... 등등을 단순 파싱

            # 기본 키(이미지) = None
            image_key = None

            if "대기중" in status_str:
                # 예: "일반매대에서 대기중"
                #     "시작점에서 대기중"
                #     "하역장(=목적지)에서 대기중"
                # station 추출
                # 문장에서 'XXX에서 대기중' 형태로 앞부분을 분리
                if "에서 대기중" in status_str:
                    # "XXX에서 대기중" 형태로 앞부분을 분리
                    parts = status_str.split("에서 대기중")
                    if len(parts) > 1:
                        st_raw = parts[0].strip()
                        st_fixed = station_map.get(st_raw, st_raw)
                        image_key = f"{r_id}{st_fixed}"  # e.g. "1일반매대"
                else:
                    # "그냥 '대기중'" 같은 형태라면, 로봇이 시작점에서 대기중이라고 가정
                    image_key = f"{r_id}시작점"

            elif "이동중" in status_str:
                # 예: "출발지에서 일반매대로 이동중"
                # station1 = 출발지, station2 = 일반
                parts = status_str.split("에서 ")
                if len(parts) == 2 and "로 이동중" in parts[1]:
                    st1_raw = parts[0].strip()
                    st2_raw = parts[1].replace("로 이동중", "").strip()
                    st1_fixed = station_map.get(st1_raw, st1_raw)
                    st2_fixed = station_map.get(st2_raw, st2_raw)
                    image_key = f"{r_id}{st1_fixed}-{st2_fixed}"

            elif "물건 담기 작업중" in status_str:
                # 예: "일반 매대에서 물건 담기 작업중"
                parts = status_str.split("에서 물건 담기 작업중")
                if len(parts) > 1:
                    st_raw = parts[0].strip()  # "일반 매대"
                    # station_map은 "일반매대" 형태로 되어 있으므로, 공백 제거:
                    st_raw_no_space = st_raw.replace(" ", "")  # "일반매대"
                    st_fixed = station_map.get(st_raw_no_space, st_raw_no_space)
                    image_key = f"{r_id}{st_fixed}"

            elif "주문처리 시작" in status_str:
                # 로봇이 막 할당된 시점 => 실제론 '출발지'에 있다고 가정
                # => "시작점" 이미지
                image_key = f"{r_id}시작점"

            # 그 외 상황에서 특정 키를 못찾으면 기본적으로 시작점 등으로 처리할 수도 있음
            # 여기서는 단순히 None이면 무시
            if image_key:
                label_widget = self.station_label_map.get(image_key)
                if label_widget:
                    label_widget.show()

    def _hide_all_map_labels(self):
        """
        map_2 ~ map_43을 전부 hide
        """
        for i in range(2, 44):
            lbl = self.findChild(QLabel, f"map_{i}")
            if lbl:
                lbl.hide()

    # ----------------------------------------------------------------
    # (6) 로봇 위치 갱신 (기존)
    # ----------------------------------------------------------------
    def update_robot_locations(self):
        """
        1초마다 /api/robots/locations 로봇 위치 갱신
        (기존에 robot_image5,6,7을 이동)
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
        좌표 변환 예시
        """
        scale = 1000
        px = int(x * scale) + (-75 + 70)
        py = int(y * scale) + (-75 + 70)
        #print(px, ",", py)
        return px, py

    # ----------------------------------------------------------------
    # (7) 로봇 상태 텍스트박스 세팅 헬퍼
    # ----------------------------------------------------------------
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

# --------------------------------------------------------------------
# 메인 실행부
# --------------------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    admin_window = AdminWindow()
    admin_window.show()
    sys.exit(app.exec_())
