# task_manager_py/adapters/ros/mobile_robot_action_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.subscription import Subscription
from std_msgs.msg import Float32
from typing import Callable, Optional
from datetime import datetime

from task_manager.action import NavDelivery
from task_manager_py.infrastructure.db_manager import DBManager
from task_manager_py.domain.models.robot import Robot

class MobileRobotActionClient(Node):
    """
    주행 로봇 액션 클라이언트.
    - 로봇 서버(NavDelivery)에 이동 Goal 전송
    - 배터리 레벨 Subscribe
    - 클라이언트 단에서 로그(deli_bot_logs)에 INSERT
    """
    def __init__(self, robot_id: str, robot_obj: Robot, db: DBManager):
        super().__init__(f"robot_{robot_id}_action_client")
        self.robot_id = robot_id
        self.robot_obj = robot_obj  # 도메인 모델
        self.db = db                # DBManager

        # (1) 액션 클라이언트
        self._nav_action_client = ActionClient(
            self,
            NavDelivery,
            f"/robot_{robot_id}/navigation_task"
        )

        # (2) 배터리 Subscribe
        self._battery_sub: Subscription = self.create_subscription(
            Float32,
            f"/robot_{robot_id}/battery",
            self._battery_callback,
            10
        )

    def navigate_to_station(
        self,
        station: str,
        done_cb: Optional[Callable[[bool], None]] = None
    ):
        """
        주행 로봇을 station으로 이동시키는 비동기 요청
        """
        # 로그: 주문 수행 시작
        self._save_mobile_log(
            status=f"주문 수행 시작 -> {station}",
            location=f"x:{self.robot_obj.location[0]:.2f}, y:{self.robot_obj.location[1]:.2f}"
        )

        goal_msg = NavDelivery.Goal()
        goal_msg.stations = [station]

        self.get_logger().info(f"[{self.robot_id}_client] Sending navigation goal -> {station}")

        if not self._nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Navigation action server not available!")
            # 주문 수행 실패 로그
            self._save_mobile_log(status="서버 연결 실패", location="N/A")
            if done_cb:
                done_cb(False)
            return

        send_goal_future = self._nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb
        )
        send_goal_future.add_done_callback(
            lambda future: self._nav_goal_response_callback(future, station, done_cb)
        )

    def _battery_callback(self, msg: Float32):
        """
        배터리 레벨 Subscribe 콜백
        """
        new_level = msg.data
        self.robot_obj.battery_level = new_level
        # 필요 시 로깅하거나, 특정 임계치 미만 시 동작 트리거 등을 추가 가능

    def _nav_feedback_cb(self, feedback_msg):
        """
        이동 중 피드백 콜백
        """
        feedback = feedback_msg.feedback

        # geometry_msgs/PoseStamped에서 pose 정보를 추출
        current_pose_stamped = feedback.current_pose
        current_position = current_pose_stamped.pose.position

        # 로봇의 위치를 실제로 갱신
        self.robot_obj.location = (current_position.x, current_position.y)

        # 로그 출력 (예: header.frame_id, 거리 등)
        self.get_logger().info(
            f"[{self.robot_id}_client] Navigation Feedback: "
            f"frame_id={current_pose_stamped.header.frame_id}, "
            f"position=({current_position.x:.2f}, {current_position.y:.2f}), "
            f"dist={feedback.distance_remaining}"
        )

    def _nav_goal_response_callback(self, future, station: str, done_cb: Optional[Callable[[bool], None]]):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Navigation goal to {station} rejected!")
            self._save_mobile_log(status="Goal Rejected", location="N/A")
            if done_cb:
                done_cb(False)
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda r: self._nav_result_callback(r, station, done_cb)
        )

    def _nav_result_callback(self, future, station: str, done_cb: Optional[Callable[[bool], None]]):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"[{self.robot_id}_client] Navigation finished: {result.error_msg}")
            self._save_mobile_log(
                status=f"주문 수행 완료 -> {station}",
                location=f"x:{self.robot_obj.location[0]:.2f}, y:{self.robot_obj.location[1]:.2f}"
            )
            if done_cb:
                done_cb(True)
        else:
            self.get_logger().error(f"Navigation failed: {result.error_msg}")
            self._save_mobile_log(status="Navigation Failed", location="N/A")
            if done_cb:
                done_cb(False)

    def _save_mobile_log(self, status: str, location: str):
        """
        deli_bot_logs에 클라이언트 단에서 INSERT
        """
        if not self.db:
            return
        sql = """
        INSERT INTO deli_bot_logs (robot_id, status, location, time)
        VALUES (%s, %s, %s, %s)
        """
        import datetime
        now_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # robot_id가 int 형태여야 한다고 가정
        rid_int = int(self.robot_id)
        self.db.execute_query(sql, (rid_int, status, location, now_str))
