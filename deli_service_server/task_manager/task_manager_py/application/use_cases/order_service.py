# task_manager_py/application/use_cases/order_service.py

import time
import threading
from task_manager_py.domain.services.route_planner import ga_optimize_order
from task_manager_py.adapters.ros.mobile_robot_action_client import MobileRobotActionClient
from task_manager_py.adapters.ros.station_manipulator_client import StationManipulatorClient
from task_manager_py.domain.models.order import Order

# 아이템 1개당 20초
ITEM_TIME_FACTOR = 20

class OrderService:
    def __init__(self, robots, occupied_info, db):
        self.robots = robots
        self.occupied_info = occupied_info
        self.order_queue = []
        self.db = db

        # 로봇별 액션 클라이언트 초기화
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

        # 2) 사용 가능한 주행 로봇 (busy=False) 중 배터리가 가장 높은 로봇 선택
        available = [r for r in self.robots.values()
                     if (r.robot_type == "주행" and not r.busy)]
        if not available:
            self.order_queue.append(order)
            return None

        available.sort(key=lambda r: r.battery_level, reverse=True)
        robot = available[0]

        # 3) GA 최적 경로
        path, cost = ga_optimize_order(
            order.station_list,
            self.occupied_info,
            robot.battery_level
        )
        print(f"[OrderService] Robot {robot.robot_id} assigned to order {order.order_id}, path={path}, cost={cost}")

        # 4) 로봇 상태 업데이트
        robot.busy = True
        robot.path_queue = path
        robot.remaining_items = dict(order.cart)
        robot.carrying_items = {}

        # 5) 별도 스레드로 주문 수행
        t = threading.Thread(
            target=self.execute_order_in_thread,
            args=(robot, order)
        )
        t.daemon = True
        t.start()

        return robot.robot_id

    def execute_order_in_thread(self, robot, order: Order):
        client = self.robot_clients[robot.robot_id]

        def finalize_robot():
            # 로봇 해제 + 남은 주문 할당
            robot.busy = False
            robot.path_queue = []
            robot.remaining_items = {}
            robot.carrying_items = {}

            if self.order_queue:
                next_order = self.order_queue.pop(0)
                self.assign_order(next_order)

        def process_stations():
            if not robot.path_queue:
                # 모두 방문하면 목적지 -> 출발지
                client.navigate_to_station("목적지", done_cb=_on_destination_done)
                return

            st = robot.path_queue[0]

            # (1) 현재 매대가 이미 점유 중인지 확인
            r_occ, r_time = self.occupied_info[st]["robot"]
            p_occ, p_time = self.occupied_info[st]["person"]

            total_occ = (r_occ or p_occ)
            total_remain = max(r_time, p_time)

            if total_occ or total_remain > 0:
                # 점유 중 -> GA 재계획
                leftover_stations = robot.path_queue[:]
                new_path, cost = ga_optimize_order(
                    leftover_stations,
                    self.occupied_info,
                    robot.battery_level
                )
                print(f"[OrderService] Station {st} is occupied({total_remain} sec). Re-planning route...")
                print(f"  old_path={leftover_stations}, new_path={new_path}, cost={cost}")

                # (2) 새 경로가 기존 경로와 완전히 동일하다면 무한 루프 위험
                if new_path == leftover_stations:
                    # 잠시 대기 후 다시 시도 (또는 다른 정책 추가 가능)
                    print("@@@@ time sleeping 2second")
                    time.sleep(2)
                    # 사람이 점유 시간을 줄이도록
                    self.reduce_occupied_time()
                    # 다시 process_stations() 시도
                    process_stations()
                    return
                else:
                    # 경로 갱신 후 재귀 호출
                    robot.path_queue = new_path
                    process_stations()
                    return

            # (3) 점유가 아니면, 이동 "시작" 시점에 매대를 점유 상태로 만든다
            station_products = order.station_items_map.get(st, {})
            total_item_count = sum(station_products.values())
            remain_time = total_item_count * ITEM_TIME_FACTOR

            # 로봇 점유 설정(이동 시작 시점)
            self.update_robot_occupied_info(st, True, remain_time)
            print(f"[OrderService] Robot {robot.robot_id} => moving to {st}, occupant_time={remain_time}s")

            # 실제 이동
            client.navigate_to_station(st, done_cb=lambda ok: _on_navigate_done(ok, st))

        def _on_navigate_done(success: bool, station: str):
            if not success:
                finalize_robot()
                return

            # 로봇이 station에 도착한 상태
            manip_client = self.manipulator_clients.get(station)
            if not manip_client:
                # 로봇팔이 없는 매대라면 -> 곧바로 점유 해제 + 종료
                self.update_robot_occupied_info(station, False, 0)
                finalize_robot()
                return

            station_products = order.station_items_map.get(station, {})
            manip_client.manipulate_station(
                station,
                station_products,
                done_cb=lambda ok: _on_manip_done(ok, station)
            )

        def _on_manip_done(success: bool, station: str):
            # 매대에서 아이템 담기가 끝남 -> 점유 해제
            self.update_robot_occupied_info(station, False, 0)

            if not success:
                finalize_robot()
                return

            # 로봇 상태 갱신
            for prod_id, qty in order.station_items_map[station].items():
                if prod_id in robot.remaining_items:
                    robot.remaining_items[prod_id] -= qty
                    if robot.remaining_items[prod_id] <= 0:
                        del robot.remaining_items[prod_id]
                robot.carrying_items[prod_id] = \
                    robot.carrying_items.get(prod_id, 0) + qty

            # 방문 완료 -> path_queue에서 제거
            if robot.path_queue and robot.path_queue[0] == station:
                robot.path_queue.pop(0)

            # 다음 스테이션 진행
            process_stations()

        def _on_destination_done(success: bool):
            if success:
                # 목적지 방문 후 -> 출발지로 이동
                client.navigate_to_station("출발지", done_cb=_on_return_home_done)
            else:
                finalize_robot()

        def _on_return_home_done(success: bool):
            if success:
                try:
                    sql = "UPDATE orders SET order_status=%s WHERE order_id=%s"
                    self.db.execute_query(sql, ("completed", order.order_id))
                    print(f"Order {order.order_id} completed.")
                except Exception as e:
                    print(f"[ERROR] Update order fail: {e}")
            finalize_robot()

        # 프로세스 시작
        process_stations()

    def update_robot_occupied_info(self, station, occupied, remain_time):
        """
        이동 시작 시점(혹은 원하는 시점)에 해당 station을 로봇으로 점유/해제
        """
        if station not in self.occupied_info:
            return
        self.occupied_info[station]["robot"] = (occupied, remain_time)

    def reduce_occupied_time(self):
        """
        주기적으로 remain_time 을 1씩 감소.
        0 이하가 되면 occupied=False 로 설정.
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
                    print(self.occupied_info)

            # 사람 점유
            p_occ, p_time = occupant_dict["person"]
            if p_occ and p_time > 0:
                new_t = p_time - 1
                if new_t <= 0:
                    occupant_dict["person"] = (False, 0)
                else:
                    occupant_dict["person"] = (True, new_t)
