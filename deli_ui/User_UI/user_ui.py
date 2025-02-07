import sys
import requests
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QDialog
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5 import uic
from PyQt5.QtGui import QPixmap

# login.ui 에서 자동 생성되는 UI 클래스
LoginUIClass = uic.loadUiType("login.ui")[0]

# mainPage.ui 에서 자동 생성되는 UI 클래스
MainUIClass = uic.loadUiType("mainPage.ui")[0]

# userInfo.ui 파일을 로드 (위에서 제시된 UI XML 내용을 .ui 파일로 저장했다고 가정)
MyPageUIClass = uic.loadUiType("my_menu.ui")[0]


class MyPage(QDialog, MyPageUIClass):
    def __init__(self, user_id=None):
        super().__init__()
        self.setupUi(self)
        self.user_id = user_id

        # 페이지 로드 시 사용자 정보, 주문 내역 불러오기
        self.loadUserInfo()
        self.loadOrders()

        # "수정하기" 버튼
        self.pushButton.clicked.connect(self.updateUser)
        # "탈퇴하기" 버튼
        self.pushButton_2.clicked.connect(self.deleteUser)

    def loadUserInfo(self):
        """GET /api/users/{userId} 호출하여 유저 정보 가져온 뒤 UI에 표시"""
        if not self.user_id:
            return

        url = f"http://localhost:8000/api/users/{self.user_id}"
        try:
            resp = requests.get(url)
            if resp.status_code == 200:
                data = resp.json()
                # 예: { "name": "홍길동", "id": "user01", "address": "서울", "email": "hong@example.com" }
                self.name.setText(data.get("name", ""))
                self.ID.setText(data.get("id", ""))
                self.address.setText(data.get("address", ""))
                self.email.setText(data.get("email", ""))
                # PW는 서버에서 직접 제공하지 않는 경우가 많으므로 여기서는 빈칸 처리
                self.PW.setText("")
            else:
                QMessageBox.warning(self, "에러", "유저 정보를 불러올 수 없습니다.")
        except Exception as e:
            QMessageBox.critical(self, "에러", f"유저 정보 조회 중 오류가 발생했습니다.\n{str(e)}")

    def loadOrders(self):
        """
        GET /api/orders?userId={userId} 로 주문 내역 받아온 뒤 스크롤 영역에 표시.
        status가 'queued'인 주문에만 취소 버튼 표시.
        """
        if not self.user_id:
            return

        url = f"http://localhost:8000/api/orders?userId={self.user_id}"
        try:
            resp = requests.get(url)
            if resp.status_code == 200:
                data = resp.json()
                orders = data.get("orders", [])

                # 스크롤 에어리어 내부 레이아웃 새로 구성
                layout = QVBoxLayout(self.scrollAreaWidgetContents)

                for order in orders:
                    order_id = order.get("orderId", "")
                    status = order.get("status", "")
                    items = order.get("items", {})
                    price = order.get("price", 0)

                    # 주문 정보를 표시할 라벨
                    text = (
                        f"주문번호: {order_id}\n"
                        f"상태: {status}\n"
                        f"아이템: {items}\n"
                        f"가격: {price}원\n"
                    )
                    label = QLabel(text)
                    label.setStyleSheet("font-size:14px; margin-bottom: 10px;")

                    # QHBoxLayout으로 라벨 + (필요 시) 버튼 가로 배치
                    hbox = QHBoxLayout()
                    hbox.addWidget(label)

                    # status가 "queued"이면 주문 취소 버튼 생성
                    if status == "queued":
                        cancel_btn = QPushButton("주문 취소")
                        cancel_btn.setStyleSheet("font-size:14px; padding:5px;")
                        # 람다를 이용하여 order_id를 인자로 넘긴다
                        cancel_btn.clicked.connect(lambda _, oid=order_id: self.cancelOrder(oid))
                        hbox.addWidget(cancel_btn)

                    # hbox를 메인 layout에 추가
                    layout.addLayout(hbox)

                self.scrollAreaWidgetContents.setLayout(layout)
            else:
                QMessageBox.warning(self, "에러", "주문 내역을 불러올 수 없습니다.")
        except Exception as e:
            QMessageBox.critical(self, "에러", f"주문 내역 조회 중 오류가 발생했습니다.\n{str(e)}")

    def cancelOrder(self, order_id):
        """
        주문 취소: DELETE /api/orders/{orderId}
        성공 시 '주문 취소 완료' 메시지 후, 주문 내역 다시 갱신
        """
        url = f"http://localhost:8000/api/orders/{order_id}"
        try:
            resp = requests.delete(url)
            if resp.status_code == 200:
                res_json = resp.json()
                status = res_json.get("status", "")
                message = res_json.get("message", "No message.")
                if status == "success":
                    QMessageBox.information(self, "주문 취소", message)
                    # 주문 목록 다시 불러오기
                    self.loadOrders()
                else:
                    QMessageBox.warning(self, "실패", message)
            else:
                QMessageBox.warning(self, "오류", "주문 취소에 실패했습니다.")
        except Exception as e:
            QMessageBox.critical(self, "에러", f"주문 취소 중 오류가 발생했습니다.\n{str(e)}")

    def updateUser(self):
        """
        POST /api/users/{userId} 로 유저 정보 수정 요청
        (name, address, email 만 수정한다고 가정)
        """
        if not self.user_id:
            return

        url = f"http://localhost:8000/api/users/{self.user_id}"
        data = {
            "name": self.name.text(),
            "address": self.address.text(),
            "email": self.email.text()
        }

        try:
            resp = requests.post(url, json=data)
            if resp.status_code == 200:
                res_json = resp.json()
                status = res_json.get("status", "")
                message = res_json.get("message", "No message.")
                if status == "success":
                    QMessageBox.information(self, "성공", message)
                else:
                    QMessageBox.warning(self, "실패", message)
            else:
                QMessageBox.warning(self, "오류", "유저 정보 수정에 실패했습니다.")
        except Exception as e:
            QMessageBox.critical(self, "에러", f"유저 정보 수정 중 오류가 발생했습니다.\n{str(e)}")

    def deleteUser(self):
        """
        DELETE /api/users/{userId} 로 유저 삭제
        성공 시 로그인 페이지로 돌아간다고 가정
        """
        if not self.user_id:
            return

        url = f"http://localhost:8000/api/users/{self.user_id}"
        try:
            resp = requests.delete(url)
            if resp.status_code == 200:
                res_json = resp.json()
                status = res_json.get("status", "")
                message = res_json.get("message", "No message.")
                if status == "success":
                    QMessageBox.information(self, "탈퇴 완료", message)
                    # 창 닫고 -> 로그인 창으로 이동 (예시로 현재 창만 닫는 로직)
                    self.close()
                else:
                    QMessageBox.warning(self, "실패", message)
            else:
                QMessageBox.warning(self, "오류", "유저 탈퇴에 실패했습니다.")
        except Exception as e:
            QMessageBox.critical(self, "에러", f"유저 삭제 중 오류가 발생했습니다.\n{str(e)}")




class MainPage(QMainWindow, MainUIClass):
    def __init__(self, user_id=None):
        super().__init__()
        self.setupUi(self)

        # user_id를 받아서 클래스 내 변수로 저장
        self.user_id = user_id

        # 라벨에 user_id 표시
        if self.user_id:
            self.user_id_.setText(f"로그인에 성공했습니다.\nID: {self.user_id}")
        else:
            self.user_id_.setText("로그인 사용자 정보가 없습니다.")

        #이미지 설정
        pixmap = QPixmap("/home/truman/github/deli_deli/src/deli_ws/deli_ui/User_UI/img/1.jpg")
        print("Pixmap1 is null?", pixmap.isNull())
        self.image_1.setPixmap(pixmap)
        self.image_1.setScaledContents(True)

        pixmap = QPixmap("/home/truman/github/deli_deli/src/deli_ws/deli_ui/User_UI/img/2.jpg")
        print("Pixmap2 is null?", pixmap.isNull())
        self.image_2.setPixmap(pixmap)
        self.image_2.setScaledContents(True)

        pixmap = QPixmap("/home/truman/github/deli_deli/src/deli_ws/deli_ui/User_UI/img/3.png")
        print("Pixmap2 is null?", pixmap.isNull())
        self.image_3.setPixmap(pixmap)
        self.image_3.setScaledContents(True)

        pixmap = QPixmap("/home/truman/github/deli_deli/src/deli_ws/deli_ui/User_UI/img/4.jpg")
        print("Pixmap2 is null?", pixmap.isNull())
        self.image_4.setPixmap(pixmap)
        self.image_4.setScaledContents(True)

        pixmap = QPixmap("/home/truman/github/deli_deli/src/deli_ws/deli_ui/User_UI/img/5.jpg")
        print("Pixmap2 is null?", pixmap.isNull())
        self.image_5.setPixmap(pixmap)
        self.image_5.setScaledContents(True)

        pixmap = QPixmap("/home/truman/github/deli_deli/src/deli_ws/deli_ui/User_UI/img/6.jpg")
        print("Pixmap2 is null?", pixmap.isNull())
        self.image_6.setPixmap(pixmap)
        self.image_6.setScaledContents(True)


        # SpinBox 초기 값 설정
        self.count1.setValue(0)
        self.count2.setValue(0)
        self.count3.setValue(0)
        self.count4.setValue(0)
        self.count5.setValue(0)
        self.count6.setValue(0)

        # 장바구니 추가 버튼 클릭 시 각 SpinBox의 값을 +1
        self.add_1.clicked.connect(lambda: self.incrementSpinbox(self.count1))
        self.add_2.clicked.connect(lambda: self.incrementSpinbox(self.count2))
        self.add_3.clicked.connect(lambda: self.incrementSpinbox(self.count3))
        self.add_4.clicked.connect(lambda: self.incrementSpinbox(self.count4))
        self.add_5.clicked.connect(lambda: self.incrementSpinbox(self.count5))
        self.add_6.clicked.connect(lambda: self.incrementSpinbox(self.count6))

        # SpinBox 값이 변경될 때마다 총 금액 업데이트
        self.count1.valueChanged.connect(self.updateTotal)
        self.count2.valueChanged.connect(self.updateTotal)
        self.count3.valueChanged.connect(self.updateTotal)
        self.count4.valueChanged.connect(self.updateTotal)
        self.count5.valueChanged.connect(self.updateTotal)
        self.count6.valueChanged.connect(self.updateTotal)

        # 주문 취소 버튼, 결제 버튼 기능 연결
        self.cancel.clicked.connect(self.cancelOrder)
        self.purchase.clicked.connect(self.purchaseOrder)

        # 프로그램 실행 시점에서 총 금액 한 번 갱신
        self.updateTotal()

        self.my_menu.clicked.connect(self.openMyPage)

    def openMyPage(self):
        """마이페이지 창 열기"""
        self.my_page = MyPage(user_id=self.user_id)
        self.my_page.show()

    def incrementSpinbox(self, spinbox):
        """해당 SpinBox 값을 1 증가"""
        spinbox.setValue(spinbox.value() + 1)

    def updateTotal(self):
        """SpinBox들의 값을 바탕으로 총 금액을 계산 후 라벨에 표시"""
        price1 = 1200  # 일반1
        price2 = 1500  # 일반2
        price3 = 3000  # 신선1
        price4 = 3500  # 신선2
        price5 = 2000  # 냉동1
        price6 = 2500  # 냉동2

        total_price = (self.count1.value() * price1 +
                       self.count2.value() * price2 +
                       self.count3.value() * price3 +
                       self.count4.value() * price4 +
                       self.count5.value() * price5 +
                       self.count6.value() * price6)

        self.total.setText(f"총액 : {total_price} 원")

    def cancelOrder(self):
        """모든 수량 SpinBox를 0으로 초기화하고 총액 갱신"""
        self.count1.setValue(0)
        self.count2.setValue(0)
        self.count3.setValue(0)
        self.count4.setValue(0)
        self.count5.setValue(0)
        self.count6.setValue(0)
        self.updateTotal()
        QMessageBox.information(self, "주문 취소", "장바구니가 초기화되었습니다.")

    def purchaseOrder(self):
        """
        현재 SpinBox 값들을 바탕으로 주문 생성 API 호출.
        (SpinBox값이 0인 경우에는 cart에서 제외)
        """
        url = "http://localhost:8000/api/orders"  # 실제 서버 주소로 변경

        # 아이템 목록 및 SpinBox 매핑
        items = [
            ("일반1", self.count1.value()),
            ("일반2", self.count2.value()),
            ("신선1", self.count3.value()),
            ("신선2", self.count4.value()),
            ("냉동1", self.count5.value()),
            ("냉동2", self.count6.value())
        ]

        # 0이 아닌 값만 cart에 담기
        cart_data = {item_name: qty for item_name, qty in items if qty > 0}

        data = {
            "userId": self.user_id,
            "cart": cart_data
        }

        # cart 안에 아무 것도 없다면(모두 0이면) 바로 안내
        if not cart_data:
            QMessageBox.warning(self, "주문 불가", "장바구니에 담긴 상품이 없습니다.")
            return

        try:
            response = requests.post(url, json=data)
            if response.status_code == 200:
                # 예: { "status": "assigned", "orderId": "order123", "message": "Order created successfully" }
                res_json = response.json()
                status = res_json.get("status", "")
                message = res_json.get("message", "응답 메시지가 없습니다.")

                if status == "assigned":
                    QMessageBox.information(self, "주문 완료", message)
                    # 주문 완료 후 장바구니 초기화 등 추가 동작이 필요하다면 아래에 작성
                    self.cancelOrder()
                else:
                    QMessageBox.warning(self, "주문 실패", message)
            else:
                QMessageBox.warning(self, "주문 오류", "주문 생성에 실패했습니다.")
        except Exception as e:
            QMessageBox.critical(self, "에러", f"주문 처리 중 오류가 발생했습니다.\n{str(e)}")


class LoginWindow(QMainWindow, LoginUIClass):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Login")

        # 로그인 버튼 클릭 시 handleLogin() 실행
        self.LoginButton.clicked.connect(self.handleLogin)

    def handleLogin(self):
        user_id = self.ID.text()
        password = self.PW.text()

        url = "http://localhost:8000/api/login"  # 실제 서버 주소로 변경
        data = {
            "userId": user_id,
            "password": password
        }
        try:
            # 서버에 로그인 POST 요청
            response = requests.post(url, json=data)

            if response.status_code == 200:
                # 로그인 성공 시 메인 페이지 띄우면서 user_id를 인자로 넘김
                self.main_page = MainPage(user_id)
                self.main_page.show()

                # 현재 창 닫기
                self.close()
            else:
                QMessageBox.warning(self, "로그인 실패", "아이디 또는 비밀번호를 확인해주세요.")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"로그인 요청 중 문제가 발생했습니다.\n{str(e)}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    login_window = LoginWindow()
    login_window.show()
    sys.exit(app.exec_())
