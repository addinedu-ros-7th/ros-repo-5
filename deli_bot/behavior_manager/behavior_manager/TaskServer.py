import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from geometry_msgs.msg import PoseStamped
from task_manager_msgs.action import Delivery
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

================================================================ """


class TaskServer(Node):
    def __init__(self, robot_id: str):
        super().__init__(f"{robot_id}_task_server")
        self.robot_id = robot_id

        # Initialize action Server
        # self.action_server = ActionServer(
        #     self, Delivery, f"{robot_id}_get_task", self.handle_dispatch_task
        # )

        # Initialize service client
        self.task_client = self.create_client(
            GetStationWaypoints, f"/{self.robot_id}/get_task_station")
        
        # Wait for the action server to be available
        while not self.task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"/{self.robot_id}/get_task_station Waiting for service...")
        self.get_logger().info(f"/{self.robot_id}/get_task_station Service is available!")

        asyncio.run(self.request_task_station())

    async def request_task_station(self):
        request = GetStationWaypoints.Request()
        request.pickups = [
            PickUp(station='apple_shelf', handler=self.robot_id, payload=[Payload(sku='apple', quantity=5)]),
            PickUp(station='peach_shelf', handler=self.robot_id, payload=[Payload(sku='peach', quantity=3)]),
        ]
        
        req_log_message = format_pickup_tasks_log(self.robot_id, request.pickups)
        self.get_logger().info(f"/{self.robot_id}/get_task_station Request sent: \n{req_log_message}")

        future = self.task_client.call_async(request)
        response = await future
        self.get_logger().info(f"/{self.robot_id}/get_task_stationResponse Response received:")

        # res_log_message = format_station_waypoints_log(self.robot_id, )
        # self.get_logger().info(f"/{self.robot_id}/get_task_stationResponse Response received: {res_log_message}")


    # TODO: Implement task service server
    def handle_dispatch_task(self, goal_handle):
        """
        Main callback for processing Task.
        """
        self.get_logger().info(f"[{self.robot_id}_task_server] Received task: {goal_handle.request.stations}")

        # Initialize result
        result = Delivery.Result()
        result.success = True
        result.error_code = 0
        result.error_msg = ""

        # Process each station in the stations list
        for station in goal_handle.request.stations:
            # Get Waypoint from Traffic Manager
            self.get_logger().info(f"Fetching waypoint for station: {station}")
            waypoint = self.traffic_client.get_waypoint(station)

    def send_feedback(self, current_pose, distance_remaining):
        """
        Send feedback to the Task Manager.
        """
        # bool success
        # int32 error_code :0=success, 1=waypoint failure, 2=navigation failure 
        # string error_msg

        feedback = Delivery.Feedback()
        feedback.current_pose = current_pose    # /amcl_pose or /tf
        feedback.distance_remaining = distance_remaining  # Update with real distance
        self.goal_handle.publish_feedback(feedback)

    def send_result(self, success, error_code=0, error_msg=""):
        """
        Send result to the Task Manager.
        """
        result = Delivery.Result()
        result.success = success
        result.error_code = error_code
        result.error_msg = error_msg
        if self.goal_handle:
            if success:
                self.goal_handle.succeed()
            else:
                self.goal_handle.abort()

        # Finalize result
        if result.success:
            self.goal_handle.succeed()
            self.get_logger().info(f"Task completed successfully: {self.goal_handle.request.stations}")
        else:
            self.get_logger().error(f"Task failed: {result.error_msg}")

        return result


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