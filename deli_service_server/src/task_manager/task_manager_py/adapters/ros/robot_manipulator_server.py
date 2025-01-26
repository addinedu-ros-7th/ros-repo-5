# task_manager_py/adapters/ros/robot_manipulator_server.py

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from task_manager.action import ManipulationTask

class RobotManipulatorServer(Node):
    def __init__(self, robot_id: str):
        super().__init__(f"{robot_id}_manipulator_server")
        self.robot_id = robot_id
        
        self._action_server = ActionServer(
            self,
            ManipulationTask,
            f"/{robot_id}/manipulation_task",
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        station = goal_handle.request.target_station
        self.get_logger().info(f"[{self.robot_id} Manipulator] Start picking at station: {station}")

        feedback_msg = ManipulationTask.Feedback()
        success = True

        for i in range(3):
            time.sleep(1)
            feedback_msg.progress = float((i+1)/3.0)*100.0
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = ManipulationTask.Result()
        result.success = success
        result.error_code = 0
        result.error_msg = f"Completed picking at {station}"
        self.get_logger().info(f"[{self.robot_id} Manipulator] Done.")
        return result
