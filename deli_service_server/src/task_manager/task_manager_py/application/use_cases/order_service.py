# task_manager_py/application/use_cases/order_service.py

import threading
from task_manager_py.domain.services.route_planner import ga_optimize_order
from task_manager_py.adapters.ros.robot_action_client import RobotActionClient

class OrderService:
    def __init__(self, robots, occupied_info):
        self.robots = robots
        self.occupied_info = occupied_info
        self.order_queue = []

        self.robot_clients = {}
        for robot_id in robots:
            client_node = RobotActionClient(robot_id)
            self.robot_clients[robot_id] = client_node

    def assign_order(self, order):
        station_list = []
        for product_name in order.cart.keys():
            if product_name.startswith("냉동"):
                if "냉동" not in station_list:
                    station_list.append("냉동")
            elif product_name.startswith("신선"):
                if "신선" not in station_list:
                    station_list.append("신선")
            else:
                if "일반" not in station_list:
                    station_list.append("일반")
        order.station_list = station_list

        available = [r for r in self.robots.values() if not r.busy]
        if not available:
            self.order_queue.append(order)
            return None

        available.sort(key=lambda r: r.battery_level, reverse=True)
        robot = available[0]

        path, cost = ga_optimize_order(
            station_list,
            self.occupied_info,
            robot.battery_level
        )
        print(f"[OrderService] {robot.robot_id} assigned order, path={path}, cost={cost}")

        robot.busy = True
        robot.path_queue = path

        # 비동기로 실행(쓰레드) 시작
        t = threading.Thread(target=self.execute_order_in_thread, args=(robot, order))
        t.daemon = True
        t.start()

        return robot.robot_id

    def execute_order_in_thread(self, robot, order):
        """
        예시: path_queue에 있는 스테이션들을 순차적으로 navigate->manipulate->다음 station
        → 끝나면 "목적지" navigate 후 robot 해제
        → 대기중인 주문 있으면 할당
        """
        client = self.robot_clients[robot.robot_id]
        stations = robot.path_queue

        # 첫 스테이션부터 순차적으로 조작하기 위한 재귀(혹은 중첩) 콜백 함수를 만든다.
        def process_stations(idx: int):
            if idx >= len(stations):
                # 모든 스테이션 처리완료 → 목적지로 이동
                client.navigate_to_station("목적지", done_cb=self._on_destination_done)
                return

            st = stations[idx]
            # 1) navigate
            client.navigate_to_station(
                st,
                done_cb=lambda success: self._on_navigate_done(success, st, idx)
            )

        def finalize_robot():
            """ 모든 작업 마친 뒤 로봇 상태 해제 + 다음 주문 할당 """
            robot.busy = False
            robot.path_queue = []
            if self.order_queue:
                next_order = self.order_queue.pop(0)
                self.assign_order(next_order)

        def _on_destination_done(success: bool):
            # 목적지 이동 끝
            print(f"[OrderService] Robot {robot.robot_id} => 목적지 이동 완료 여부: {success}")
            finalize_robot()

        def _on_navigate_done(success: bool, st: str, idx: int):
            if not success:
                print(f"[OrderService] Robot {robot.robot_id} navigation failed to {st}")
                finalize_robot()
                return

            # navigate 성공 → station occupied
            self.update_occupied_info(st, True, remain_time=0)

            # 2) pick up
            client.manipulate_station(
                st,
                done_cb=lambda success2: self._on_manipulate_done(success2, st, idx)
            )

        def _on_manipulate_done(success: bool, st: str, idx: int):
            if not success:
                print(f"[OrderService] Robot {robot.robot_id} manipulation failed at {st}")
                self.update_occupied_info(st, False, 0)
                finalize_robot()
                return

            self.update_occupied_info(st, False, 0)
            process_stations(idx + 1)

        # 함수들을 인스턴스 메서드에 바인딩
        self._on_destination_done = _on_destination_done
        self._on_navigate_done = _on_navigate_done
        self._on_manipulate_done = _on_manipulate_done

        # 0번 스테이션부터 시작
        process_stations(0)

    def update_occupied_info(self, station, occupied, remain_time):
        self.occupied_info[station] = (occupied, remain_time)

    def reduce_occupied_time(self):
        for st, (o, t) in self.occupied_info.items():
            if o and t > 0:
                nt = t - 1
                if nt <= 0:
                    self.occupied_info[st] = (False, 0)
                else:
                    self.occupied_info[st] = (True, nt)
