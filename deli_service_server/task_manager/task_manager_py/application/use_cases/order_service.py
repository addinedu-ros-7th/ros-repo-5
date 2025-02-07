# task_manager_py/application/use_cases/order_service.py

import time
import threading
from task_manager_py.domain.services.route_planner import ga_optimize_order
from task_manager_py.adapters.ros.mobile_robot_action_client import MobileRobotActionClient
from task_manager_py.adapters.ros.station_manipulator_client import StationManipulatorClient
from task_manager_py.domain.models.order import Order
from datetime import datetime
import socket
from struct import *

import logging

logger = logging.getLogger(__name__)

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
        
        # try:
        #     self.sock = socket.socket()
        #     self.sock.connect(("192.168.0.2", 80))
        #     print(f"@@@@@@{self.sock}")
        # except Exception as e:
        #     print("소켓 연결 중 예외 발생:", e)

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
        # 필요한 경우 current_station 초기화 (이미 기본값이 '출발지'일 수 있음)
        robot.current_station = "출발지"
        # 작업 시작 전 activity
        robot.current_activity = "주문 처리 시작"

        # try :
        #     # 전자석 붙이기
        #     data = pack ("ii",25, True)
        #     self.sock.send(data)
        # except Exception as e:
        #     print("전자석 붙이기중 예외 발생", e)
        
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
            robot.current_station = "출발지"
            robot.current_activity = "대기중"

            if self.order_queue:
                next_order = self.order_queue.pop(0)
                self.assign_order(next_order)

        def is_station_occupied(station):
            """해당 스테이션이 로봇 또는 사람이 점유 중인지 여부와 남은 시간 체크"""
            if station not in self.occupied_info:
                return False, 0
            r_occ, r_time = self.occupied_info[station]["robot"]
            p_occ, p_time = self.occupied_info[station]["person"]
            return (r_occ or p_occ), max(r_time, p_time)

        def process_stations():
            """
            스테이션별로 순차 이동. path_queue에 남아있다면 계속 스테이션으로 이동함.
            모든 스테이션 방문 후에는 '목적지' -> '출발지' 로 이동.
            """
            if not robot.path_queue:
                # 모두 방문하면 목적지 -> 출발지
                robot.current_activity = f"{robot.current_station}에서 목적지로 이동중"
                client.navigate_to_station("목적지", {}, done_cb=_on_destination_done)
                return

            st = robot.path_queue[0]

            # 매대 점유 여부 확인
            total_occ, total_remain = is_station_occupied(st)
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

                # (2) '새 경로 == 기존 경로' 이거나 
                #     '새 경로의 첫 번째 스테이션이 여전히 점유 중'이면 2초 대기 후 다시 재계산
                if (new_path == leftover_stations) or \
                   (len(new_path) > 0 and is_station_occupied(new_path[0])[0]):
                    print("@@@@ time sleeping 2second")
                    time.sleep(2)
                    self.reduce_occupied_time()
                    process_stations()
                    return
                else:
                    robot.path_queue = new_path
                    process_stations()
                    return

            # 여기까지 왔다면 점유가 아니므로 이동 시작
            station_products = order.station_items_map.get(st, {})
            total_item_count = sum(station_products.values())
            remain_time = total_item_count * ITEM_TIME_FACTOR

            # 로봇 점유 설정(이동 시작 시점)
            self.update_robot_occupied_info(st, True, remain_time)
            print(f"[OrderService] Robot {robot.robot_id} => moving to {st}, occupant_time={remain_time}s")

            # 이동을 시작하기 전에 activity 업데이트
            robot.current_activity = f"{robot.current_station}에서 {st}로 이동중"
            # 실제 이동 (주행 액션 호출)
            client.navigate_to_station(st, station_products, done_cb=lambda ok: _on_navigate_done(ok, st))

        def _on_navigate_done(success: bool, station: str):
            if not success:
                finalize_robot()
                return

            # 로봇이 station에 도착했으므로 current_station 갱신
            robot.current_station = station
            robot.current_activity = f"{station} 도착(대기중)"

            # 로봇팔(매니퓰레이터) 유무 확인
            manip_client = self.manipulator_clients.get(station)
            if not manip_client:
                # 로봇팔이 없는 매대라면 -> 곧바로 점유 해제 + 종료
                self.update_robot_occupied_info(station, False, 0)
                finalize_robot()
                return

            # 로봇팔 작업 시작 전 상태 업데이트
            robot.current_activity = f"{station} 매대에서 물건 담기 작업중"
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

            # 로봇 상태 갱신 (적재 아이템)
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

            # 매대 조작 완료 후 대기중 상태로 갱신
            robot.current_activity = f"{station}에서 대기중"
            # 다음 스테이션 진행
            process_stations()

        def _on_destination_done(success: bool):
            if success:
                # 목적지 도착 => current_station 갱신
                robot.current_station = "목적지"
                robot.current_activity = "목적지 도착(대기중)"

                # # 전자석 떼기
                # try:
                #     data = pack("ii", 25, False)
                #     self.sock.send(data)
                # except Exception as e:
                #     print("전자석 붙이기중 예외 발생", e)

                # 목적지 방문 후 -> 출발지로 이동
                robot.current_activity = f"{robot.current_station}에서 출발지로 이동중"
                client.navigate_to_station(f"{robot.robot_id}출발지", {}, done_cb=_on_return_home_done)
            else:
                finalize_robot()

        def _on_return_home_done(success: bool):
            if success:
                robot.current_station = "출발지"
                robot.current_activity = "출발지 도착(대기중)"

                try:
                    sql = """
                    UPDATE orders
                    SET order_status=%s,
                        over_at=%s
                    WHERE order_id=%s
                    """
                    self.db.execute_query(sql, ("completed", datetime.now(), order.order_id))
                    print(f"Order {order.order_id} completed.")
                except Exception as e:
                    print(f"[ERROR] Update order fail: {e}")
            finalize_robot()

        # 프로세스 시작
        robot.current_activity = "주문처리 시작"
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

            # 사람 점유
            p_occ, p_time = occupant_dict["person"]
            if p_occ and p_time > 0:
                new_t = p_time - 1
                if new_t <= 0:
                    occupant_dict["person"] = (False, 0)
                else:
                    occupant_dict["person"] = (True, new_t)
