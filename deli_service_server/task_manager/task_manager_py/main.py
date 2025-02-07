#!/usr/bin/env python3
# task_manager_py/main.py

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

import uvicorn
from fastapi import FastAPI

from task_manager_py.infrastructure.db_manager import DBManager
from task_manager_py.domain.models.robot import Robot
from task_manager_py.application.use_cases.order_service import OrderService
from task_manager_py.adapters.ros.robot_navigation_server import RobotNavigationServer
from task_manager_py.adapters.ros.robot_manipulator_server import RobotManipulatorServer

# ------------------------------------------------------------
# 전역 영역에서 FastAPI 인스턴스 선언
# ------------------------------------------------------------
app: FastAPI = None

def background_occupy(order_service_instance: OrderService):
    """주기적으로 station의 remain_time(occupied_time)을 감소시키는 쓰레드."""
    print(">>> background_occupy thread started!")
    while True:
        # OrderService 내 reduce_occupied_time()에서
        # 'robot', 'person' 각각의 remain_time을 감소
        order_service_instance.reduce_occupied_time()
        time.sleep(1)

def ros_spin(all_nodes):
    """
    ROS2 노드들을 병렬 처리(MultiThreadedExecutor)로 spin.
    """
    print(">>> ros_spin thread started!")
    executor = MultiThreadedExecutor()
    for node in all_nodes:
        executor.add_node(node)
    print(">>> Executor spin() ...")
    try:
        executor.spin()
    finally:
        for node in all_nodes:
            node.destroy_node()
        rclpy.shutdown()
        print(">>> rclpy shutdown done.")

def create_app() -> FastAPI:
    """
    FastAPI 인스턴스를 생성하고 필요한 리소스를 주입.
    """
    print(">>> create_app() called!")
    app = FastAPI()

    # -------------------------
    # 1) DB 초기화
    # -------------------------
    try:
        db = DBManager(host="localhost", user="root", password="0000", db="deli")
        app.state.db = db
        print(">>> DBManager created successfully.")
    except Exception as e:
        print(f"!!! DBManager creation failed: {e}")
        raise

    # -------------------------
    # 2) 로봇 세팅 (주행 3대 + 로봇팔 3대)
    # -------------------------
    robots = {
        "1": Robot("1", "주행로봇1", 1.0, robot_type="주행", loaction= (-0.09,-0.403)),
        "2": Robot("2", "주행로봇2", 1.0, robot_type="주행", loaction= (-0.06,0.0)),
        "3": Robot("3", "주행로봇3", 1.0, robot_type="주행", loaction= (0.0,0.0)),
        "10": Robot("10", "냉동로봇팔", 1.0, robot_type="로봇팔"),
        "11": Robot("11", "신선로봇팔", 1.0, robot_type="로봇팔"),
        "12": Robot("12", "일반로봇팔", 1.0, robot_type="로봇팔"),
    }
    print(f">>> Created robots: {list(robots.keys())}")

    # -------------------------
    # 3) 매대 점유 정보 구조
    #    로봇/사람 각각 기록
    # -------------------------
    occupied_info = {
        "냉동":  {"robot": (False, 0), "person": (False, 0)},
        "신선":  {"robot": (False, 0), "person": (False, 0)},
        "일반":  {"robot": (False, 0), "person": (False, 0)}
    }

    # -------------------------
    # 4) OrderService 생성
    # -------------------------
    service = OrderService(robots, occupied_info, db)
    app.state.order_service = service
    print(">>> OrderService created and set to app.state.order_service")

    # -------------------------
    # 5) FastAPI 라우터 등록
    # -------------------------
    from task_manager_py.adapters.fastapi.fastapi_server import router
    app.include_router(router)
    print(">>> Router included!")

    # -------------------------
    # 6) 백그라운드 쓰레드
    #    station remain_time 감소
    # -------------------------
    threading.Thread(target=background_occupy, args=(service,), daemon=True).start()

    return app

def main():
    """
    ros2 run task_manager main
    """
    print(">>> main() called!")
    # 1) rclpy 초기화
    rclpy.init()
    print(">>> rclpy.init() done.")

    # 2) ROS2 액션 서버 생성
    #print(">>> Creating RobotNavigationServer for 1,2,3...")
    #nav_server_r1 = RobotNavigationServer("1")
    #nav_server_r2 = RobotNavigationServer("2")
    #nav_server_r3 = RobotNavigationServer("3")

    print(">>> Creating RobotManipulatorServer for cold,fresh,normal...")
    manip_server_cold = RobotManipulatorServer("manipulator_cold")
    manip_server_fresh = RobotManipulatorServer("manipulator_fresh")
    manip_server_normal = RobotManipulatorServer("manipulator_normal")

    # 3) FastAPI 앱 생성(전역 변수 app)
    global app
    app = create_app()

    # 디버그 정보
    order_service = app.state.order_service
    print(f">>> order_service.robots keys: {list(order_service.robots.keys())}")
    print(f">>> order_service.occupied_info: {order_service.occupied_info}")

    # 4) OrderService 안에 있는 클라이언트 노드들 (MobileRobotActionClient, StationManipulatorClient)
    client_nodes = list(order_service.robot_clients.values())
    manipulator_nodes = list(order_service.manipulator_clients.values())

    # 5) 모든 노드를 합쳐 spin
    all_nodes = [
        #nav_server_r1,
        #nav_server_r2,
        #nav_server_r3,
        manip_server_cold,
        manip_server_fresh,
        manip_server_normal
    ] + client_nodes + manipulator_nodes

    # 6) ROS2 spin을 백그라운드 쓰레드에서 실행
    ros_thread = threading.Thread(
        target=ros_spin,
        args=(all_nodes,),
        daemon=True
    )
    ros_thread.start()
    print(">>> ros_thread started!")

    # 7) uvicorn으로 FastAPI 서버 실행
    print(">>> Starting uvicorn server on 0.0.0.0:8000 ...")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

if __name__ == "__main__":
    main()
