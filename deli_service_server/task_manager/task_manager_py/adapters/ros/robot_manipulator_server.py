# task_manager_py/adapters/ros/robot_manipulator_server.py

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from task_manager_msgs.action import DispatchManipulationTask

class RobotManipulatorServer(Node):
    def __init__(self, manipulator_id: str):
        super().__init__(f"{manipulator_id}_server")
        self.manipulator_id = manipulator_id

        self._action_server = ActionServer(
            self,
            DispatchManipulationTask,
            f"/{manipulator_id}/manipulation_task",
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        station = goal_handle.request.target_station
        item_names = goal_handle.request.item_names
        item_quantities = goal_handle.request.item_quantities

        self.get_logger().info(
            f"[{self.manipulator_id}] Start picking station={station}, "
            f"items={list(zip(item_names,item_quantities))}"
        )

        feedback_msg = DispatchManipulationTask.Feedback()
        success = True

        # 예시: 3초간 동작을 가정
        total_steps = 3
        for i in range(total_steps):
            time.sleep(1)
            feedback_msg.progress = float((i+1) / total_steps) * 100.0
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = DispatchManipulationTask.Result()
        result.success = success
        result.error_code = 0
        result.error_msg = f"Completed picking items at {station}"
        return result
