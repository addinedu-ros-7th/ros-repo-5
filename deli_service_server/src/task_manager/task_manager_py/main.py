#!/usr/bin/env python3
# task_manager_py/main.py

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

import uvicorn
from fastapi import FastAPI

# DB, 도메인
from task_manager_py.infrastructure.db_manager import DBManager
from task_manager_py.domain.models.robot import Robot
from task_manager_py.application.use_cases.order_service import OrderService

# ROS 액션 서버
from task_manager_py.adapters.ros.robot_navigation_server import RobotNavigationServer
from task_manager_py.adapters.ros.robot_manipulator_server import RobotManipulatorServer

def background_occupy(order_service_instance: OrderService):
    while True:
        order_service_instance.reduce_occupied_time()
        time.sleep(1)


def ros_spin(all_nodes):
    executor = MultiThreadedExecutor()
    for node in all_nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        for node in all_nodes:
            node.destroy_node()
        rclpy.shutdown()


def create_app() -> FastAPI:
    app = FastAPI()

    # 1) DBManager
    db = DBManager(host="localhost", user="root", password="0000", db="deli")
    app.state.db = db

    # 2) 로봇 + 매대 → OrderService 생성
    robots = {
        "robot1": Robot("robot1", "Robot1", 1.0),
        "robot2": Robot("robot2", "Robot2", 1.0),
        "robot3": Robot("robot3", "Robot3", 1.0)
    }
    occupied_info = {"냉동": (False, 0), "신선": (False, 0), "일반": (False, 0)}
    service = OrderService(robots, occupied_info)
    app.state.order_service = service

    # 3) FastAPI 라우터
    from task_manager_py.adapters.fastapi.fastapi_server import router
    app.include_router(router)

    # 4) 주기적으로 remain_time 감소
    threading.Thread(target=background_occupy, args=(service,), daemon=True).start()

    return app


def main():
    rclpy.init()

    # 액션 서버 3대 (네비게이션 + 매니퓰레이션)
    nav_server_r1 = RobotNavigationServer("robot1")
    nav_server_r2 = RobotNavigationServer("robot2")
    nav_server_r3 = RobotNavigationServer("robot3")

    manip_server_r1 = RobotManipulatorServer("robot1")
    manip_server_r2 = RobotManipulatorServer("robot2")
    manip_server_r3 = RobotManipulatorServer("robot3")

    app = create_app()
    order_service = app.state.order_service

    # OrderService 내부 로봇 클라이언트 노드
    client_nodes = list(order_service.robot_clients.values())

    # 모든 노드를 하나의 Executor에서 spin
    all_nodes = [
        nav_server_r1, nav_server_r2, nav_server_r3,
        manip_server_r1, manip_server_r2, manip_server_r3
    ] + client_nodes

    ros_thread = threading.Thread(
        target=ros_spin,
        args=(all_nodes,),
        daemon=True
    )
    ros_thread.start()

    # FastAPI
    uvicorn.run(app, host="0.0.0.0", port=8000)

    # 종료 시 DB 연결 정리
    app.state.db.close()


if __name__ == "__main__":
    main()
