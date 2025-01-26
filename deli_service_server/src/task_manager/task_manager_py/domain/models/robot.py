# task_manager_py/domain/models/robot.py

class Robot:
    def __init__(self, robot_id: str, name: str, battery_level: float):
        self.robot_id = robot_id
        self.name = name
        self.battery_level = battery_level
        self.current_station = "출발지"
        self.busy = False
        self.path_queue = []