# task_manager_py/domain/models/order.py

class Order:
    def __init__(self, user_id: str, order_id: str, cart: dict):
        self.user_id = user_id
        self.order_id = order_id
        self.cart = cart
        self.order_status = ""
        self.station_list = None