# task_manager_py/adapters/ros/station_manipulator_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from typing import Callable, Optional, Dict
from datetime import datetime

from task_manager_msgs.action import DispatchManipulationTask
from task_manager_py.infrastructure.db_manager import DBManager

class StationManipulatorClient(Node):
    """
    매대 로봇팔 액션 클라이언트
    """
    def __init__(self, manipulator_id: str, db: DBManager = None):
        """
        :param manipulator_id: 예) 'manipulator_cold'
        :param db: DBManager
        """
        super().__init__(f"{manipulator_id}_client")
        self.manipulator_id = manipulator_id
        self.db = db

        self._manip_action_client = ActionClient(
            self,
            DispatchManipulationTask,
            f"/{manipulator_id}/manipulation_task"
        )

    def manipulate_station(
        self,
        station: str,
        items: Dict[str, int],
        done_cb: Optional[Callable[[bool], None]] = None
    ):
        """
        조작(물건 담기) 비동기 액션 Goal 전송
        """
        self._save_arm_log(status=f"주문 수행 시작({station})")

        goal_msg = DispatchManipulationTask.Goal()
        goal_msg.target_station = station
        goal_msg.item_names = list(items.keys())
        goal_msg.item_quantities = list(items.values())

        self.get_logger().info(f"[{self.manipulator_id}_client] Sending manipulation goal -> station={station}")

        if not self._manip_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Manipulation server not available!")
            self._save_arm_log(status="서버 연결 실패")
            if done_cb:
                done_cb(False)
            return

        send_goal_future = self._manip_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._manip_feedback_cb
        )
        send_goal_future.add_done_callback(
            lambda future: self._manip_goal_response_callback(future, station, done_cb)
        )

    def _manip_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[{self.manipulator_id}] progress={feedback.progress}%")

    def _manip_goal_response_callback(self, future, station: str, done_cb: Optional[Callable[[bool], None]]):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Manipulation goal for {station} rejected!")
            self._save_arm_log(status=f"Goal Rejected({station})")
            if done_cb:
                done_cb(False)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda r: self._manip_result_callback(r, station, done_cb)
        )

    def _manip_result_callback(self, future, station: str, done_cb: Optional[Callable[[bool], None]]):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"[{self.manipulator_id} Client] Completed picking items at {station}")
            self._save_arm_log(status=f"주문 수행 완료({station})")
            if done_cb:
                done_cb(True)
        else:
            self.get_logger().error(f"[{self.manipulator_id}] Failed: {result.error_msg}")
            self._save_arm_log(status=f"Manipulation Failed({station})")
            if done_cb:
                done_cb(False)

    def _save_arm_log(self, status: str):
        """
        deli_arm_logs에 INSERT
        """
        if not self.db:
            return
        sql = """
        INSERT INTO deli_arm_logs (robot_id, status, time)
        VALUES (%s, %s, %s)
        """
        now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # 예시 변환
        rid_int = self._convert_manipulator_id_to_int(self.manipulator_id)
        self.db.execute_query(sql, (rid_int, status, now_str))

    def _convert_manipulator_id_to_int(self, manip_id: str) -> int:
        # 예시 매핑
        if manip_id == "manipulator_cold":
            return 10
        elif manip_id == "manipulator_fresh":
            return 11
        elif manip_id == "manipulator_normal":
            return 12
        return 0
