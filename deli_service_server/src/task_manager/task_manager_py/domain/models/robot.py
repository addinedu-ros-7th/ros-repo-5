# task_manager_py/domain/models/robot.py

class Robot:
    def __init__(self, robot_id: str, name: str, battery_level: float, robot_type: str = "주행"):
        self.robot_id = robot_id       # 예) '1'
        self.name = name              # "주행로봇1" 등
        self.battery_level = battery_level  # 0.0 ~ 1.0
        self.robot_type = robot_type  # "주행" 혹은 "로봇팔"

        # --- 작업 상태 ---
        self.remaining_items = {}     # 아직 담지 않은 아이템
        self.carrying_items = {}      # 로봇이 현재 적재 중인 아이템
        self.location = (0.0, 0.0)    # (x, y)

        self.current_station = "출발지"
        self.busy = False
        self.path_queue = []

        self.waiting_time = 0
