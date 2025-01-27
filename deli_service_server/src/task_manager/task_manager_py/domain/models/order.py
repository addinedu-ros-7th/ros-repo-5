# task_manager_py/domain/models/order.py

class Order:
    def __init__(self, user_id: str, order_id: str, cart: dict):
        self.user_id = user_id
        self.order_id = order_id
        self.cart = cart
        self.order_status = ""
        self.station_list = None
        
        # [NEW] 스테이션별로 어떤 제품을 몇 개 담아야 하는지 저장할 딕셔너리
        # 예: {"냉동": {"냉동1": 2, "냉동2": 1}, "신선": {...}, ...}
        self.station_items_map = {}