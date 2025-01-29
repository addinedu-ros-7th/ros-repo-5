# task_manager_py/application/use_cases/order_service.py

import threading
from task_manager_py.domain.services.route_planner import ga_optimize_order
from task_manager_py.adapters.ros.mobile_robot_action_client import MobileRobotActionClient
from task_manager_py.adapters.ros.station_manipulator_client import StationManipulatorClient
from task_manager_py.domain.models.order import Order

# 필요하다면 아래 상수 정의를 제거하거나, 다른 용도로 사용할 수도 있음
# ROBOT_OCCUPIED_PENALTY = 20

# 예: 아이템 1개당 5초
ITEM_TIME_FACTOR = 5

class OrderService:
    def __init__(self, robots, occupied_info, db):
        """
        :param robots: { "1": Robot, "2": Robot, ... }
        :param occupied_info: {
            "냉동": {
                "robot": (False, 0),
                "person": (False, 0)
            },
            "신선": {
                "robot": (False, 0),
                "person": (False, 0)
            },
            "일반": {
                "robot": (False, 0),
                "person": (False, 0)
            }
        }
        :param db: DBManager
        """
        self.robots = robots
        self.occupied_info = occupied_info
        self.order_queue = []
        self.db = db

        # 로봇별 Action Client 초기화
        self.robot_clients = {}
        for r_id, robot_obj in robots.items():
            if robot_obj.robot_type == "주행":
                client_node = MobileRobotActionClient(r_id, robot_obj, self.db)
                self.robot_clients[r_id] = client_node

        # 매대 -> 매니퓰레이터 매핑
        self.manipulator_mapping = {
            "냉동": "manipulator_cold",
            "신선": "manipulator_fresh",
            "일반": "manipulator_normal"
        }
        self.manipulator_clients = {}
        for station, manip_id in self.manipulator_mapping.items():
            m_node = StationManipulatorClient(manip_id, db=self.db)
            self.manipulator_clients[station] = m_node

    def assign_order(self, order: Order):
        """
        새 주문이 들어왔을 때 가능한 주행 로봇을 할당하고, 주문을 비동기로 수행하는 스레드 시작.
        """
        # 1) 스테이션별 아이템 분류
        station_items_map = {}
        for product_id, qty in order.cart.items():
            if product_id.startswith("냉동"):
                st = "냉동"
            elif product_id.startswith("신선"):
                st = "신선"
            else:
                st = "일반"
            station_items_map.setdefault(st, {})
            station_items_map[st][product_id] = qty

        order.station_items_map = station_items_map
        order.station_list = list(station_items_map.keys())

        # 2) 사용 가능한 주행 로봇 (busy가 아닌) 중 배터리가 가장 높은 로봇 선택
        available = [r for r in self.robots.values() if (r.robot_type == "주행" and not r.busy)]
        if not available:
            # 로봇 할당 불가 -> 큐에 대기
            self.order_queue.append(order)
            return None

        available.sort(key=lambda r: r.battery_level, reverse=True)
        robot = available[0]

        # 3) GA 최적 경로 계산
        path, cost = ga_optimize_order(
            order.station_list,
            self.occupied_info,
            robot.battery_level
        )

        print(f"[OrderService] Robot {robot.robot_id} assigned to order {order.order_id}, path={path}, cost={cost}")

        # 로봇 상태 업데이트
        robot.busy = True
        robot.path_queue = path  # 처음 방문할 매대 목록
        robot.remaining_items = dict(order.cart)
        robot.carrying_items = {}

        # 4) 주문 처리 스레드 실행
        t = threading.Thread(target=self.execute_order_in_thread, args=(robot, order))
        t.daemon = True
        t.start()

        return robot.robot_id

    def execute_order_in_thread(self, robot, order: Order):
        """
        실제 주문 수행 로직 (별도 스레드).
        - 매대 방문 전, 점유 상태를 확인하여 점유 중이면 GA로 재경로 편성
        - 매대 조작(Manipulation) 수행
        - 모든 매대 방문 후 목적지 -> 출발지 이동
        - 주문 상태 완료 처리
        """
        client = self.robot_clients[robot.robot_id]

        def finalize_robot():
            """
            로봇 상태 리셋 + 대기중인 다음 주문 할당 시도
            """
            robot.busy = False
            robot.path_queue = []
            robot.remaining_items = {}
            robot.carrying_items = {}

            if self.order_queue:
                next_order = self.order_queue.pop(0)
                self.assign_order(next_order)

        def process_stations():
            """
            남아있는 매대(robot.path_queue)를 순차 방문.
            다음 매대가 다른 로봇/사람으로 점유 중이면 GA 재실행 후 경로 재편성.
            """
            if not robot.path_queue:
                # 더 방문할 매대가 없으면 목적지 -> 출발지 후 종료
                client.navigate_to_station("목적지", done_cb=_on_destination_done)
                return

            next_station = robot.path_queue[0]

            # 다음 매대가 점유 중인지 확인
            r_occ, r_time = self.occupied_info[next_station]["robot"]
            p_occ, p_time = self.occupied_info[next_station]["person"]

            total_occ = (r_occ or p_occ)
            total_remain = max(r_time, p_time)

            if total_occ or total_remain > 0:
                # 매대가 점유 중이라면 남은 스테이션들에 대해 GA 재실행
                print(f"[OrderService] Station {next_station} is occupied({total_remain} sec), Re-planning route...")
                leftover_stations = robot.path_queue[:]  # 아직 방문 못한 매대들

                new_path, cost = ga_optimize_order(
                    leftover_stations,
                    self.occupied_info,
                    robot.battery_level
                )
                robot.path_queue = new_path
                print(f" [Re-plan] new path={new_path}, cost={cost}")

                # 새 경로로 다시 방문 시도
                process_stations()
                return

            # 점유가 아니면 이동 시작
            st = next_station
            client.navigate_to_station(st, done_cb=lambda ok: _on_navigate_done(ok, st))

        def _on_navigate_done(success: bool, station: str):
            if not success:
                finalize_robot()
                return

            # 매대에서 담아야 할 아이템의 총 개수를 기반으로 점유 시간 계산
            station_products = order.station_items_map.get(station, {})
            total_item_count = sum(station_products.values())
            # 예) item_count 개당 5초
            remain_time = total_item_count * ITEM_TIME_FACTOR

            # 로봇이 해당 스테이션에 도착 -> 로봇 점유 설정
            self.update_robot_occupied_info(station, True, remain_time)

            # 매니퓰레이터
            manip_client = self.manipulator_clients.get(station)
            if not manip_client:
                # 해당 스테이션에 로봇팔이 없다면 조작 불가
                self.update_robot_occupied_info(station, False, 0)
                finalize_robot()
                return

            manip_client.manipulate_station(
                station,
                station_products,
                done_cb=lambda ok: _on_manip_done(ok, station)
            )

        def _on_manip_done(success: bool, station: str):
            # 로봇 점유 해제
            self.update_robot_occupied_info(station, False, 0)

            if not success:
                finalize_robot()
                return

            # 로봇 상태 갱신(담은 아이템 수량 등)
            for prod_id, qty in order.station_items_map[station].items():
                if prod_id in robot.remaining_items:
                    robot.remaining_items[prod_id] -= qty
                    if robot.remaining_items[prod_id] <= 0:
                        del robot.remaining_items[prod_id]
                robot.carrying_items[prod_id] = robot.carrying_items.get(prod_id, 0) + qty

            # 해당 스테이션 방문 완료 -> path_queue에서 제거
            if robot.path_queue and robot.path_queue[0] == station:
                robot.path_queue.pop(0)

            # 다음 매대 방문
            process_stations()

        def _on_destination_done(success: bool):
            """
            목적지 도착 후 -> 출발지로 이동
            """
            if success:
                client.navigate_to_station("출발지", done_cb=_on_return_home_done)
            else:
                finalize_robot()

        def _on_return_home_done(success: bool):
            """
            출발지로 복귀 완료 시 주문 완료 처리
            """
            if success:
                try:
                    sql = "UPDATE orders SET order_status=%s WHERE order_id=%s"
                    self.db.execute_query(sql, ("completed", order.order_id))
                    print(f"Order {order.order_id} completed.")
                except Exception as e:
                    print(f"[ERROR] Update order fail: {e}")

            finalize_robot()

        # 스테이션 방문 로직 시작
        process_stations()

    def update_robot_occupied_info(self, station, occupied, remain_time):
        """
        로봇에 의한 점유만 업데이트 (station: str, occupied: bool, remain_time: int)
        occupied_info[station]["robot"] = (occupied, remain_time)
        """
        if station not in self.occupied_info:
            return
        self.occupied_info[station]["robot"] = (occupied, remain_time)

    def reduce_occupied_time(self):
        """
        주기적으로 remain_time을 감소시키는 함수.
        - 로봇(robot)
        - 사람(person)
        두 항목 모두 1씩 감소. 0 이하가 되면 occupied=False
        """
        for st, occupant_dict in self.occupied_info.items():
            # 로봇 점유
            r_occ, r_time = occupant_dict["robot"]
            if r_occ and r_time > 0:
                new_t = r_time - 1
                if new_t <= 0:
                    occupant_dict["robot"] = (False, 0)
                else:
                    occupant_dict["robot"] = (True, new_t)

            # 사람 점유
            p_occ, p_time = occupant_dict["person"]
            if p_occ and p_time > 0:
                new_t = p_time - 1
                if new_t <= 0:
                    occupant_dict["person"] = (False, 0)
                else:
                    occupant_dict["person"] = (True, new_t)
