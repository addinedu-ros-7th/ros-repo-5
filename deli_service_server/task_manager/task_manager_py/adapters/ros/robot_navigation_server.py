import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

# 변경된 action, msg import
from task_manager.action import DispatchDeliveryTask
from task_manager.msg import PickUp

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
        # 변경된 Goal: PickUp[] pickups
        pickups = goal_handle.request.pickups
        self.get_logger().info(f"[Robot {self.robot_id}] NavGoal => pickup from={pickups[0].station}")

        feedback_msg = DispatchDeliveryTask.Feedback()
        success = True

        # 단순히 pickups 안의 스테이션을 순차 이동한다고 가정
        # (아래는 예시로, pickups 길이에 상관없이 5초 정도 움직인다고 가정)
        for i in range(5):
            time.sleep(1)

            # 피드백에 현재 pose, 남은 거리 등 임의로 넣어봄
            feedback_msg.current_pose = PoseStamped()
            feedback_msg.current_pose.header.frame_id = f"Moving... {i+1}/5"
            feedback_msg.distance_remaining = float(5 - (i+1))
            goal_handle.publish_feedback(feedback_msg)

        # 일단 성공 처리
        goal_handle.succeed()
        result = DispatchDeliveryTask.Result()
        result.success = success
        result.error_code = 0

        # 어떤 스테이션에 도착했는지 메시지 구성 (예: 첫 번째 Pickup의 station 기준)
        if pickups:
            st_list = [pu.station for pu in pickups]
            result.error_msg = f"Arrived at {st_list}"
        else:
            result.error_msg = "No station in pickups"

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
