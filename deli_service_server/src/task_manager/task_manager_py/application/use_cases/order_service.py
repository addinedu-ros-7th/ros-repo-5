import threading
from task_manager_py.domain.services.route_planner import ga_optimize_order
from task_manager_py.adapters.ros.mobile_robot_action_client import MobileRobotActionClient
from task_manager_py.adapters.ros.station_manipulator_client import StationManipulatorClient
from task_manager_py.domain.models.order import Order

class OrderService:
    def __init__(self, robots, occupied_info, db):
        """
        :param robots: { "1": Robot 객체, "2": Robot 객체, ... }
        :param occupied_info: {"냉동":(False,0), "신선":(False,0), ...}
        :param db: DBManager
        """
        self.robots = robots
        self.occupied_info = occupied_info
        self.order_queue = []  # 대기 중인 주문들을 관리하는 큐
        self.db = db

        # 로봇별 Action Client
        self.robot_clients = {}
        for r_id, robot_obj in robots.items():
            if robot_obj.robot_type == "주행":
                # 주행 로봇용 클라이언트
                client_node = MobileRobotActionClient(r_id, robot_obj, self.db)
                self.robot_clients[r_id] = client_node

        # 스테이션 -> 매니퓰레이터 매핑 (예시)
        self.manipulator_mapping = {
            "냉동": "manipulator_cold",
            "신선": "manipulator_fresh",
            "일반": "manipulator_normal"
        }
        # 매니퓰레이터 클라이언트
        self.manipulator_clients = {}
        for station, manip_id in self.manipulator_mapping.items():
            m_node = StationManipulatorClient(manip_id, db=self.db)
            self.manipulator_clients[station] = m_node

    def assign_order(self, order: Order):
        """
        새로운 주문(Order)이 들어왔을 때 로봇을 할당하고 수행 스레드를 실행하는 함수
        1) 주문 항목들을 스테이션별로 분류
        2) 사용가능한 주행 로봇 중에서 배터리가 가장 높은 로봇 선택
        3) GA(유전 알고리즘)로 최적 경로 계산
        4) 별도의 스레드로 주문 수행
        """
        # 1) 스테이션별 분류
        station_items_map = {}
        for product_id, qty in order.cart.items():
            # 상품 ID로부터 스테이션을 단순 분류 (예시)
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

        # 2) 사용 가능 주행 로봇 확인
        available = [r for r in self.robots.values() if (r.robot_type == "주행" and not r.busy)]
        if not available:
            # 사용 가능한 로봇이 없다면 큐에 저장
            self.order_queue.append(order)
            return None

        # 배터리 레벨이 높은 순 정렬
        available.sort(key=lambda r: r.battery_level, reverse=True)
        robot = available[0]  # 가장 배터리가 높은 로봇 선택

        # 3) GA 최적 경로 계산
        path, cost = ga_optimize_order(
            order.station_list,
            self.occupied_info,
            robot.battery_level
        )
        print(f"[OrderService] {robot.robot_id} assigned order {order.order_id}, path={path}, cost={cost}")

        # 로봇 busy 상태 전환
        robot.busy = True
        robot.path_queue = path
        robot.remaining_items = dict(order.cart)  # 아직 픽업하지 않은 아이템
        robot.carrying_items = {}

        # 4) 주문 실행 스레드 시작
        t = threading.Thread(target=self.execute_order_in_thread, args=(robot, order))
        t.daemon = True
        t.start()

        return robot.robot_id

    def execute_order_in_thread(self, robot, order: Order):
        """
        로봇을 이용해 주문 수행을 처리하는 실제 로직 (별도 스레드)
        """
        client = self.robot_clients[robot.robot_id]
        stations = robot.path_queue

        def finalize_robot():
            """
            주문 작업이 종료(성공/실패) 시 로봇 상태 초기화 및 대기 주문 처리
            """
            robot.busy = False
            robot.path_queue = []
            robot.carrying_items = {}
            robot.remaining_items = {}

            # 대기열에 주문이 있으면 다시 할당
            if self.order_queue:
                next_order = self.order_queue.pop(0)
                self.assign_order(next_order)

        def process_stations(idx: int):
            """
            순차적으로 스테이션 방문
            """
            if idx >= len(stations):
                # 모든 스테이션 방문 후 -> '목적지' 이동
                client.navigate_to_station("목적지", done_cb=_on_destination_done)
                return

            st = stations[idx]
            client.navigate_to_station(st, done_cb=lambda ok: _on_navigate_done(ok, st, idx))

        def _on_navigate_done(success: bool, st: str, idx: int):
            """
            주행 완료 콜백
            """
            if not success:
                finalize_robot()
                return
            robot.current_station = st
            self.update_occupied_info(st, True, 0)  # 스테이션 점유

            station_products = order.station_items_map.get(st, {})
            manip_client = self.manipulator_clients.get(st)
            if not manip_client:
                # 매니퓰레이터 없으면 바로 점유 해제 후 종료
                self.update_occupied_info(st, False, 0)
                finalize_robot()
                return

            manip_client.manipulate_station(
                st,
                station_products,
                done_cb=lambda ok: _on_manip_done(ok, st, idx)
            )

        def _on_manip_done(success: bool, st: str, idx: int):
            if not success:
                self.update_occupied_info(st, False, 0)
                finalize_robot()
                return

            # 로봇 객체 상태 갱신
            for prod_id, qty in order.station_items_map[st].items():
                # remaining -> carrying
                if prod_id in robot.remaining_items:
                    robot.remaining_items[prod_id] -= qty
                    if robot.remaining_items[prod_id] <= 0:
                        del robot.remaining_items[prod_id]

                if prod_id not in robot.carrying_items:
                    robot.carrying_items[prod_id] = 0
                robot.carrying_items[prod_id] += qty

            self.update_occupied_info(st, False, 0)
            process_stations(idx+1)

        # ------------------------
        #  목적지 도착 -> 출발지 복귀
        # ------------------------
        def _on_destination_done(success: bool):
            if success:
                # 목적지 도착 후, 다시 출발지로 이동
                client.navigate_to_station("출발지", done_cb=_on_return_home_done)
            else:
                finalize_robot()

        def _on_return_home_done(success: bool):
            if success:
                # 주문 완료 -> DB 상태 업데이트
                try:
                    sql = "UPDATE orders SET order_status=%s WHERE order_id=%s"
                    self.db.execute_query(sql, ("completed", order.order_id))
                    print(f"Order {order.order_id} completed and returned to start.")
                except Exception as e:
                    print(f"[ERROR] Update order fail: {e}")
            finalize_robot()

        # 스테이션 방문 시작
        process_stations(0)

    def update_occupied_info(self, station, occupied, remain_time):
        """
        스테이션 점유 정보 갱신
        """
        self.occupied_info[station] = (occupied, remain_time)

    def reduce_occupied_time(self):
        """
        주기적으로 remain_time 카운트다운
        """
        for st, (o, t) in self.occupied_info.items():
            if o and t > 0:
                nt = t - 1
                if nt <= 0:
                    self.occupied_info[st] = (False, 0)
                else:
                    self.occupied_info[st] = (True, nt)
