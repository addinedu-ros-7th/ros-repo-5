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

$ ros2 action send_goal /delibot_1_dispatch_delivery_task task_manager_msgs/action/DispatchDeliveryTask "{pickups: [{station: 'apple_shelf', handler: 'delibot_1', payload: [{sku: 'apple', quantity: 5}]}]}" --feedback

================================================================ """


class TaskServer(Node):
    def __init__(self, robot_id: str):
        super().__init__(f"{robot_id}_task_server")
        self.robot_id = robot_id

        # Initialize action Server
        self.task_server = ActionServer(
            self, DispatchDeliveryTask, f"{robot_id}_dispatch_delivery_task", self.handle_dispatch_task
        )

        # Initialize service client
        self.task_client = self.create_client(
            GetStationWaypoints, f"/{self.robot_id}/get_task_station")
        
        # Wait for the action server to be available
        while not self.task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"/{self.robot_id}/get_task_station Waiting for service...")
        self.get_logger().info(f"/{self.robot_id}/get_task_station Service is available!")


    async def handle_dispatch_task(self, goal_handle):
        """
        Receive an action goal and send a result to the Task Manager.
        """
        # request.pickups = [
        #     PickUp(station='apple_shelf', handler=self.robot_id, payload=[Payload(sku='apple', quantity=5)]),
        #     PickUp(station='peach_shelf', handler=self.robot_id, payload=[Payload(sku='peach', quantity=3)]),
        # ]

        pickups = goal_handle.request.pickups
        goal_log_message = format_pickup_tasks_log(self.robot_id, pickups)
        self.get_logger().info(f"{self.robot_id}_dispatch_delivery_task Goal received: {goal_log_message}")


        """
        result.success = 0: waypoint success & navigation success
        result.success = 1: waypoint fail
        result.success = 2: navigation fail
        """

        # Request waypoints
        station_waypoint = await self.request_task_station(pickups)
        if not station_waypoint:
            result = self.send_result(
                goal_handle, success=False, error_code=1, error_msg="Failed to get station waypoints."
            )
            return result

        # Receive Pose
        current_pose, distance_remaining = self.handle_pose()

        # Send feedbacks
        await self.send_feedback(current_pose, distance_remaining)

        # Send a result
        result = self.send_result(
            goal_handle, success=True, error_code=0, error_msg="All tasks completed successfully."
        )
        return result


    async def request_task_station(self, pickups):
        """
        Make a service request to get station waypoints and receive a response from the Traffic Client.
        """
        request = GetStationWaypoints.Request()
        request.pickups = pickups
        req_log_message = format_pickup_tasks_log(self.robot_id, pickups)
        self.get_logger().info(f"/{self.robot_id}/get_task_station Request sent: \n{req_log_message}")

        # Request station waypoints
        future = self.task_client.call_async(request)
        response = await future

        if response is None:
            self.get_logger().error(f"/{self.robot_id}/get_task_station Response is None!")
            return None
    
        if not response.station_waypoints:
            self.get_logger().error(f"/{self.robot_id}/get_task_station Failed to receive waypoints.")
            return None
        
        res_log_message = format_station_waypoints_log(self.robot_id, response.station_waypoints)
        self.get_logger().info(f"/{self.robot_id}/get_task_station Response received: {res_log_message}")
        return response.station_waypoints


    async def send_feedback(self, current_pose, distance_remaining):
        """
        Send action feedbacks to the Task Manager.
        """
        # bool success
        # int32 error_code :0=success, 1=waypoint failure, 2=navigation failure 
        # string error_msg

        feedback_msg = DispatchDeliveryTask.Feedback()
        feedback_msg.current_pose = current_pose    # /amcl_pose or /tf
        feedback_msg.distance_remaining = distance_remaining  # Update with real distance
        self.goal_handle.publish_feedback(feedback_msg)
        await asyncio.sleep(1)

    # TODO: Implement server in NavManager 
    def handle_pose(self):
        # 현재 PoseStamped 정보 생성
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 0.0

        distance_remaining = 15

        return pose, distance_remaining

    
    def send_result(self, goal_handle, success, error_code=0, error_msg=""):
        """
        Send an action result to the Task Manager.
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

        log_message = f"Task Result - Success: {success}, Error Code: {error_code}, Message: {error_msg}"

        # Finalize result
        if success:
            self.get_logger().info(f"Task completed successfully: {log_message}")
        else:
            self.get_logger().error(f"Task failed: {log_message}")

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