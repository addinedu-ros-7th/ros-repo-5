# task_manager_py/adapters/ros/robot_navigation_server.py

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from task_manager.action import NavDelivery
from geometry_msgs.msg import PoseStamped

class RobotNavigationServer(Node):
    def __init__(self, robot_id: str):
        super().__init__(f"{robot_id}_navigation_server")
        self.robot_id = robot_id
        self._action_server = ActionServer(
            self,
            NavDelivery,
            f"/{robot_id}/navigation_task",
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"[{self.robot_id} Navigation] Received goal: {goal_handle.request.stations}")

        feedback_msg = NavDelivery.Feedback()  # <-- 생성
        success = True

        # 예: feedback_msg.current_pose 는 PoseStamped
        # (전제: NavDelivery.action의 feedback 정의에 geometry_msgs/PoseStamped current_pose가 있어야 함)
        for i in range(5):
            time.sleep(1)
            feedback_msg.current_pose.header.frame_id = f"Moving... {i+1}/5"
            feedback_msg.distance_remaining = float(5 - (i+1))
            goal_handle.publish_feedback(feedback_msg)

        # 완료 시 result
        goal_handle.succeed()
        result = NavDelivery.Result()
        result.success = success
        result.error_code = 0
        result.error_msg = f"Arrived at {goal_handle.request.stations}"
        return result