import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from geometry_msgs.msg import PoseStamped
from task_manager_msgs.action import DispatchDeliveryTask
from task_manager_msgs.msg import PickUp, Payload
from traffic_manager_msgs.srv import GetStationWaypoints
from traffic_manager_msgs.msg import StationWaypoint

from traffic_manager.utils import format_pickup_tasks_log, format_station_waypoints_log

import asyncio

""" ================================================================

< Action Server >
- service type : Delivery
- service name : "get_task"
- callback :


< Delivery >
# goal
Station[] stations
---
# result
bool success

# 0=success, 1=waypoint failure, 2=navigation failure
int32 error_code
string error_msg

---
# feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining


< Test Command >

$ ros2 action send_goal /delibot_1/dispatch_delivery_task task_manager_msgs/action/DispatchDeliveryTask "{pickups: [{station: 'apple_shelf', handler: 'delibot_1', payload: [{sku: 'apple', quantity: 5}]}]}" --feedback

================================================================ """


class TaskServer(Node):
    def __init__(self, robot_id: str):
        super().__init__(f"{robot_id}_task_server")
        self.robot_id = robot_id

        # Initialize action Server
        self.task_server = ActionServer(
            self, DispatchDeliveryTask, f"{self.robot_id}/dispatch_delivery_task", self.handle_dispatch_task
        )
        self.get_logger().info(f"/{self.robot_id}/dispatch_delivery_task Service is ready!")

        # Initialize service client
        self.task_client = self.create_client(
            GetStationWaypoints, f"/{self.robot_id}/get_task_station")
        
        # Wait for the action server to be available
        while not self.task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"/{self.robot_id}/get_task_station Waiting for service...")
        self.get_logger().info(f"/{self.robot_id}/get_task_station Service is available!")


    async def handle_dispatch_task(self, goal_handle):
        """
        Handles incoming dispatch delivery task requests.
        """
        self.goal_handle = goal_handle
        pickups = goal_handle.request.pickups
        goal_log_message = format_pickup_tasks_log(self.robot_id, pickups)
        self.get_logger().info(f"/{self.robot_id}_dispatch_delivery_task Goal received: \n{goal_log_message}")

        station_waypoint = await self.request_task_station(pickups)
        if not station_waypoint:
            result = self.send_result(
                goal_handle, success=False, error_code=1, error_msg="Failed to get station waypoints."
            )
            return result

        current_pose, distance_remaining = self.handle_pose()
        await self.send_feedback(current_pose, distance_remaining)

        result = self.send_result(
            goal_handle, success=True, error_code=0, error_msg="All tasks completed successfully."
        )
        return result


    async def request_task_station(self, pickups):
        """
        Requests waypoints for pickup stations from the Traffic Manager.
        """
        request = GetStationWaypoints.Request()
        request.pickups = pickups

        future = self.task_client.call_async(request)
        response = await future

        if response is None or not response.station_waypoints:
            self.get_logger().error(f"/{self.robot_id}/get_task_station Failed to receive waypoints.")
            return None
        return response.station_waypoints


    async def send_feedback(self, current_pose, distance_remaining):
        """
        Send action feedbacks.
        """
        if hasattr(self, "goal_handle") and self.goal_handle:
            feedback_msg = DispatchDeliveryTask.Feedback()
            feedback_msg.current_pose = current_pose    # /amcl_pose or /tf
            feedback_msg.distance_remaining = distance_remaining  # Update with real distance
            self.goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f"Feedback sent: \n- Current pose: {current_pose}, Distance remaining: {distance_remaining}")
        else:
            self.get_logger().error("No active goal handle to send feedback.")

    
    def send_result(self, goal_handle, success, error_code=0, error_msg=""):
        """
        Send the final result of the action task.

        Args:
            goal_handle (GoalHandle): Handle for the current goal.
            success (bool): Whether the task was successful.
            error_code (int, optional): Error code (0=success, 1=waypoint failure, 2=navigation failure). Defaults to 0.
            error_msg (str, optional): Error message. Defaults to "".
        
        Returns:
            DispatchDeliveryTask.Result: The final result of the task execution.
        """
        result = DispatchDeliveryTask.Result()
        result.success = success
        result.error_code = error_code
        result.error_msg = error_msg

        if goal_handle:
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

        log_message = f"Task Result: \n- Success: {success}, Error Code: {error_code}, Message: {error_msg}"

        if success:
            self.get_logger().info(f"Task completed successfully: {log_message}")
        else:
            self.get_logger().error(f"Task failed: {log_message}")

        return result


    # TODO: Implement server in NavManager 
    def handle_pose(self):
        # 현재 PoseStamped 정보 생성
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 0.0

        distance_remaining = 15.0
        return pose, distance_remaining


def main(args=None):
    rclpy.init(args=args)
    node = TaskServer("delibot_1")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()