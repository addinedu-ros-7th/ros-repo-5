# task_manager_py/adapters/ros/robot_action_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from typing import Callable, Optional

from task_manager.action import NavDelivery
from task_manager.action import ManipulationTask

class RobotActionClient(Node):
    def __init__(self, robot_id: str):
        super().__init__(f"{robot_id}_action_client")
        self.robot_id = robot_id

        # Navigation / Manipulation 액션 클라이언트
        self._nav_action_client = ActionClient(
            self,
            NavDelivery,
            f"/{robot_id}/navigation_task"
        )
        self._manip_action_client = ActionClient(
            self,
            ManipulationTask,
            f"/{robot_id}/manipulation_task"
        )

    ## ----------------------
    ##  NAVIGATION
    ## ----------------------
    def navigate_to_station(
        self,
        station: str,
        done_cb: Optional[Callable[[bool], None]] = None
    ):
        """
        비동기 방식으로 특정 station으로 이동 Goal을 보낸다.
        - done_cb: 최종 성공/실패(bool)를 받는 콜백
        """
        goal_msg = NavDelivery.Goal()
        goal_msg.stations = [station]

        self.get_logger().info(f"[{self.robot_id} Client] Sending navigation goal -> {station}")

        # 서버 대기(비동기 방식이라도, 최소한 서버가 떠있는지 wait_for_server)
        if not self._nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Navigation action server not available!")
            if done_cb:
                done_cb(False)
            return

        # 1) Goal 전송 (비동기)
        send_goal_future = self._nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb
        )
        # 2) Goal 핸들 생성 시점 콜백 등록
        send_goal_future.add_done_callback(
            lambda future: self._nav_goal_response_callback(future, done_cb)
        )

    def _nav_feedback_cb(self, feedback_msg):
        """ Navigation 피드백 콜백 """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[{self.robot_id} Navigation Feedback] "
            f"Pose={feedback.current_pose.header.frame_id}, "
            f"Dist remaining={feedback.distance_remaining}"
        )

    def _nav_goal_response_callback(self, future, done_cb: Optional[Callable[[bool], None]]):
        """
        GoalHandle 수락/거절 여부가 결정되었을 때 호출
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected!")
            if done_cb:
                done_cb(False)
            return

        # goal이 수락되었으니, Result도 비동기로 받는다
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda r: self._nav_result_callback(r, done_cb)
        )

    def _nav_result_callback(self, future, done_cb: Optional[Callable[[bool], None]]):
        """
        최종 Result가 왔을 때 호출
        """
        result = future.result().result
        print("@@@ NAV result @@@", result)

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

    ## ----------------------
    ##  MANIPULATION
    ## ----------------------
    def manipulate_station(
        self,
        station: str,
        done_cb: Optional[Callable[[bool], None]] = None
    ):
        """ 비동기 조작(픽업) Goal 전송 """
        goal_msg = ManipulationTask.Goal()
        goal_msg.target_station = station

        self.get_logger().info(f"[{self.robot_id} Client] Sending manipulation goal -> {station}")

        if not self._manip_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Manipulation action server not available!")
            if done_cb:
                done_cb(False)
            return

        send_goal_future = self._manip_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._manip_feedback_cb
        )
        send_goal_future.add_done_callback(
            lambda future: self._manip_goal_response_callback(future, done_cb)
        )

    def _manip_feedback_cb(self, feedback_msg):
        """ Manipulation 피드백 콜백 """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[{self.robot_id} Manipulator Feedback] progress={feedback.progress}%"
        )

    def _manip_goal_response_callback(self, future, done_cb: Optional[Callable[[bool], None]]):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Manipulation goal rejected!")
            if done_cb:
                done_cb(False)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda r: self._manip_result_callback(r, done_cb)
        )

    def _manip_result_callback(self, future, done_cb: Optional[Callable[[bool], None]]):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"[{self.robot_id} Client] Manipulation finished: {result.error_msg}")
            if done_cb:
                done_cb(True)
        else:
            self.get_logger().error(f"Manipulation failed: {result.error_msg}")
            if done_cb:
                done_cb(False)
