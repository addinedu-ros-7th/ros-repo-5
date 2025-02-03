# task_manager_py/adapters/ros/robot_navigation_server.py

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from task_manager.action import DispatchDeliveryTask

class RobotNavigationServer(Node):
    def __init__(self, robot_id: str):
        """
        :param robot_id: 예) "1" (DB 상 PK)
        """
        super().__init__(f"robot_{robot_id}_navigation_server")
        self.robot_id = robot_id

        # DispatchDeliveryTask 액션 서버
        self._action_server = ActionServer(
            self,
            DispatchDeliveryTask,
            f"/robot_{robot_id}/navigation_task",
            self.execute_callback
        )

        # -------------------------
        # 배터리 퍼블리셔 (데모용)
        # -------------------------
        self._battery_pub = self.create_publisher(Float32, f"/robot_{robot_id}/battery", 10)

        # 타이머: 1초마다 배터리 레벨 발행 (데모용)
        self._battery_level = 1.0  # 100%
        self._battery_timer = self.create_timer(1.0, self._publish_battery_level)

    def execute_callback(self, goal_handle):
        stations = goal_handle.request.stations
        self.get_logger().info(f"[Robot {self.robot_id}] NavGoal => {stations}")

        feedback_msg = DispatchDeliveryTask.Feedback()
        success = True

        # 5초간 이동 시뮬레이션
        for i in range(5):
            time.sleep(1)
            feedback_msg.current_pose.header.frame_id = f"Moving... {i+1}/5"
            feedback_msg.distance_remaining = float(5 - (i+1))
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = DispatchDeliveryTask.Result()
        result.success = success
        result.error_code = 0
        result.error_msg = f"Arrived at {stations}"
        return result

    def _publish_battery_level(self):
        """
        배터리 레벨을 Publish하는 타이머 콜백 (데모용).
        0.01씩 감소하다가 0.5 미만이면 1.0으로 리셋.
        """
        msg = Float32()
        self._battery_level -= 0.01
        if self._battery_level < 0.5:
            self._battery_level = 1.0  # 데모 시나리오: 충전했다고 가정
        msg.data = self._battery_level
        self._battery_pub.publish(msg)
