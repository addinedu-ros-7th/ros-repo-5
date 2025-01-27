# task_manager_py/application/use_cases/order_service.py

import threading
from task_manager_py.domain.services.route_planner import ga_optimize_order
from task_manager_py.adapters.ros.mobile_robot_action_client import MobileRobotActionClient
from task_manager_py.adapters.ros.station_manipulator_client import StationManipulatorClient
from task_manager_py.domain.models.order import Order

class OrderService:
    def __init__(self, robots, occupied_info, db):
        """
        :param robots: 주행 로봇 정보 (robot_id -> Robot 객체)
        :param occupied_info: 매대 점유 정보 (예: {"냉동": (False,0), "신선": (False,0), ...})
        :param db: DB 매니저
        """
        self.robots = robots
        self.occupied_info = occupied_info
        self.order_queue = []
        self.db = db

        # -----------------------------
        # 주행 로봇 클라이언트 생성
        # -----------------------------
        self.robot_clients = {}
        for robot_id in robots:
            client_node = MobileRobotActionClient(robot_id)
            self.robot_clients[robot_id] = client_node

        # -----------------------------
        # 매대 로봇팔 클라이언트 생성
        # -----------------------------
        self.manipulator_mapping = {
            "냉동": "manipulator_cold",
            "신선": "manipulator_fresh",
            "일반": "manipulator_normal"
        }
        self.manipulator_clients = {}
        for station, manip_id in self.manipulator_mapping.items():
            m_node = StationManipulatorClient(manip_id)
            self.manipulator_clients[station] = m_node

    def assign_order(self, order: Order):
        """
        새 주문을 로봇에 할당.
        1) 스테이션별 제품 분류
        2) 현재 다른 로봇이 특정 매대를 쓰고 있으면 해당 매대를 occupied=True 로 표시  # [CHANGED]
        3) GA로 최적 경로 계산
        4) 가장 적합한 로봇에 path 할당
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

            if st not in station_items_map:
                station_items_map[st] = {}
            station_items_map[st][product_id] = qty

        order.station_items_map = station_items_map
        order.station_list = list(station_items_map.keys())

        # 2) 사용 가능 로봇 찾기
        available = [r for r in self.robots.values() if not r.busy]
        if not available:
            # 로봇이 없으면 주문 대기열에 저장
            self.order_queue.append(order)
            return None

        # 배터리 잔량이 높은 로봇을 우선할당
        available.sort(key=lambda r: r.battery_level, reverse=True)
        robot = available[0]

        # -------------------------------------------------------
        # [CHANGED]
        # "새로운 주문"에서 GA 계산하기 전,
        # 이미 다른 로봇이 어떤 매대로 이동 중이거나 매대에서 물건을 담고 있다면
        # 그 매대를 '점유된 매대'로 간주하여 cost를 높이도록 설정한다.
        #
        # 주의: self.occupied_info는 실제 매대 점유 상태를 나타내므로,
        #       원본을 직접 건드리지 않고 local 복사본을 써도 되고,
        #       혹은 그대로 써도 됩니다(아래 예시는 원본에 직접 반영).
        # -------------------------------------------------------
        for other_robot in self.robots.values():
            if other_robot.busy and other_robot != robot:
                # 2-1) 이미 어떤 매대를 담고 있는 경우 (current_station)
                #      예: current_station이 "냉동"/"신선"/"일반" 중 하나라면 occupied 처리
                if other_robot.current_station in ["냉동", "신선", "일반"]:
                    self.occupied_info[other_robot.current_station] = (True, 0)

                # 2-2) 이동 중이라면 다음에 갈 매대( path_queue[0] )를 점유 처리
                if other_robot.path_queue:
                    next_st = other_robot.path_queue[0]
                    if next_st in ["냉동", "신선", "일반"]:
                        self.occupied_info[next_st] = (True, 0)
        # -------------------------------------------------------

        # 3) GA를 통한 최적 경로
        path, cost = ga_optimize_order(
            order.station_list,
            self.occupied_info,
            robot.battery_level
        )
        print(f"[OrderService] {robot.robot_id} assigned order {order.order_id}, path={path}, cost={cost}")

        # 4) 로봇에 path 할당 + 비동기 실행
        robot.busy = True
        robot.path_queue = path

        t = threading.Thread(target=self.execute_order_in_thread, args=(robot, order))
        t.daemon = True
        t.start()

        return robot.robot_id

    def execute_order_in_thread(self, robot, order: Order):
        """
        path_queue 순서대로 이동 -> 매대 로봇팔 조작 -> ...
        """
        client = self.robot_clients[robot.robot_id]
        stations = robot.path_queue

        def process_stations(idx: int):
            if idx >= len(stations):
                # 모든 스테이션 끝 → 목적지 이동
                client.navigate_to_station("목적지", done_cb=_on_destination_done)
                return

            st = stations[idx]
            # 이동
            client.navigate_to_station(
                st,
                done_cb=lambda success: _on_navigate_done(success, st, idx)
            )

        def finalize_robot():
            """ 모든 작업 종료 후 로봇 상태 해제 + 대기 주문 처리 """
            robot.busy = False
            robot.path_queue = []
            # 대기중인 주문 있으면 다시 할당
            if self.order_queue:
                next_order = self.order_queue.pop(0)
                self.assign_order(next_order)

        def _on_destination_done(success: bool):
            print(f"[OrderService] Robot {robot.robot_id} => 목적지 이동 완료 여부: {success}")
            if success:
                # 주문 완료 시 DB 업데이트
                try:
                    update_sql = "UPDATE orders SET order_status=%s WHERE order_id=%s"
                    self.db.execute_query(update_sql, ("completed", order.order_id))
                    print(f"Order {order.order_id} is now completed in DB.")
                except Exception as e:
                    print(f"[ERROR] Updating order status failed: {e}")

            finalize_robot()

        def _on_navigate_done(success: bool, st: str, idx: int):
            if not success:
                print(f"[OrderService] Robot {robot.robot_id} navigation failed to {st}")
                finalize_robot()
                return

            # 로봇이 실제로 st에 도착했으므로 current_station 갱신
            robot.current_station = st

            # 점유 처리 (도착해서 물건 담을 동안)
            self.update_occupied_info(st, True, remain_time=0)

            # 물건 담기
            station_products = order.station_items_map.get(st, {})
            manip_client = self.manipulator_clients.get(st)
            if not manip_client:
                print(f"[OrderService] No manipulator client for station {st}!")
                self.update_occupied_info(st, False, 0)
                finalize_robot()
                return

            manip_client.manipulate_station(
                st,
                station_products,
                done_cb=lambda success2: _on_manipulate_done(success2, st, idx)
            )

        def _on_manipulate_done(success: bool, st: str, idx: int):
            if not success:
                print(f"[OrderService] Robot {robot.robot_id} manipulation failed at {st}")
                self.update_occupied_info(st, False, 0)
                finalize_robot()
                return

            # 물건 담기 끝 → 점유 해제
            self.update_occupied_info(st, False, 0)
            # 다음 스테이션
            process_stations(idx + 1)

        # 첫 스테이션부터 시작
        process_stations(0)

    def update_occupied_info(self, station, occupied, remain_time):
        self.occupied_info[station] = (occupied, remain_time)

    def reduce_occupied_time(self):
        """
        주기적으로 remain_time 1씩 감소, 0 이하가 되면 occupied=False
        """
        for st, (o, t) in self.occupied_info.items():
            if o and t > 0:
                nt = t - 1
                if nt <= 0:
                    self.occupied_info[st] = (False, 0)
                else:
                    self.occupied_info[st] = (True, nt)
