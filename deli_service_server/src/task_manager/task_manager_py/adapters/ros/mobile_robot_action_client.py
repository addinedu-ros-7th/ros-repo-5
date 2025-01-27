# task_manager_py/adapters/ros/mobile_robot_action_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from typing import Callable, Optional

# 이동(Navigation)에만 사용
from task_manager.action import NavDelivery

class MobileRobotActionClient(Node):
    """
    주행 로봇 전용 액션 클라이언트 (이동 기능만 담당).
    robot_id는 'robot1', 'robot2', 'robot3' 등 주행 로봇의 고유 ID.
    """
    def __init__(self, robot_id: str):
        super().__init__(f"{robot_id}_action_client")
        self.robot_id = robot_id

        # ---------------------------
        #  Navigation ActionClient
        # ---------------------------
        self._nav_action_client = ActionClient(
            self,
            NavDelivery,
            f"/{robot_id}/navigation_task"
        )

    def navigate_to_station(
        self,
        station: str,
        done_cb: Optional[Callable[[bool], None]] = None
    ):
        """
        주행 로봇을 station으로 이동시키는 비동기 요청.
        :param station: 이동해야 할 스테이션(문자열)
        :param done_cb: 이동 완료 후 호출될 콜백 (인자: bool 성공여부)
        """
        # 1) Goal 메시지 생성
        goal_msg = NavDelivery.Goal()
        goal_msg.stations = [station]

        self.get_logger().info(f"[{self.robot_id} Client] Sending navigation goal -> {station}")

        # 2) 서버가 준비됐는지 확인
        if not self._nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Navigation action server not available!")
            if done_cb:
                done_cb(False)
            return

        # 3) Goal 전송 (비동기)
        send_goal_future = self._nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb
        )
        # 4) Goal 요청 결과 콜백 등록
        send_goal_future.add_done_callback(
            lambda future: self._nav_goal_response_callback(future, done_cb)
        )

    def _nav_feedback_cb(self, feedback_msg):
        """
        네비게이션 작업의 진행 상황 피드백
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[{self.robot_id} Navigation Feedback] "
            f"Pose={feedback.current_pose.header.frame_id}, "
            f"Dist remaining={feedback.distance_remaining}"
        )

    def _nav_goal_response_callback(self, future, done_cb: Optional[Callable[[bool], None]]):
        """
        서버가 Goal을 수락 혹은 거부한 직후의 콜백
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected!")
            if done_cb:
                done_cb(False)
            return

        # 수락된 경우 이동 완료 결과 기다리기
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda r: self._nav_result_callback(r, done_cb)
        )

    def _nav_result_callback(self, future, done_cb: Optional[Callable[[bool], None]]):
        """
        네비게이션 액션 결과 도착 시 콜백
        """
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f"[{self.robot_id} Client] Navigation finished: {result.error_msg}"
            )
            if done_cb:
                done_cb(True)
        else:
            self.get_logger().error(f"Navigation failed: {result.error_msg}")
            if done_cb:
                done_cb(False)
