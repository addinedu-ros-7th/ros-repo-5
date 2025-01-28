# task_manager_py/domain/models/order.py

class Order:
    def __init__(self, user_id: str, order_id: str, cart: dict):
        self.user_id = user_id
        self.order_id = order_id
        self.cart = cart
        self.order_status = ""

        # 역 내 스테이션 방문 목록, 아이템 맵
        self.station_list = []
        self.station_items_map = {}
