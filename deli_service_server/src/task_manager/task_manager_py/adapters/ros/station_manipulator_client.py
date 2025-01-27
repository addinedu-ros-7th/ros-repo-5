# task_manager_py/adapters/ros/station_manipulator_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from typing import Callable, Optional, Dict

from task_manager.action import ManipulationTask

class StationManipulatorClient(Node):
    """
    매대 로봇팔 전용 액션 클라이언트 (조작 기능만 담당).
    매대별로 다른 manipulator_id를 갖는다.
    예) manipulator_cold, manipulator_fresh, manipulator_normal
    """
    def __init__(self, manipulator_id: str):
        super().__init__(f"{manipulator_id}_client")
        self.manipulator_id = manipulator_id

        # Manipulation 액션 클라이언트
        self._manip_action_client = ActionClient(
            self,
            ManipulationTask,
            f"/{manipulator_id}/manipulation_task"
        )

    def manipulate_station(
        self,
        station: str,
        items: Dict[str, int],
        done_cb: Optional[Callable[[bool], None]] = None
    ):
        """
        조작(물건 담기 등) 비동기 액션 Goal 전송
        :param station: 실제 매대 이름 (냉동, 신선, 일반)
        :param items: 담아야 할 아이템 { "냉동1":2, ... }
        :param done_cb: 완료 콜백
        """
        goal_msg = ManipulationTask.Goal()
        goal_msg.target_station = station
        goal_msg.item_names = list(items.keys())
        goal_msg.item_quantities = list(items.values())

        self.get_logger().info(f"[{self.manipulator_id} Client] Sending manipulation goal -> station={station}, items={items}")

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
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[{self.manipulator_id} Manipulator Feedback] progress={feedback.progress}%"
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
            self.get_logger().info(f"[{self.manipulator_id} Client] Manipulation finished: {result.error_msg}")
            if done_cb:
                done_cb(True)
        else:
            self.get_logger().error(f"Manipulation failed: {result.error_msg}")
            if done_cb:
                done_cb(False)
