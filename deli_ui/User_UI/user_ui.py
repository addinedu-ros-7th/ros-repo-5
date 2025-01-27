import sys
import requests
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5 import uic

# login.ui 에서 자동 생성되는 UI 클래스
LoginUIClass = uic.loadUiType("login.ui")[0]

# mainPage.ui 에서 자동 생성되는 UI 클래스
MainUIClass = uic.loadUiType("mainPage.ui")[0]


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
