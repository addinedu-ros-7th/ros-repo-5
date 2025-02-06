import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import PoseStamped
from task_manager_msgs.action import DispatchDeliveryTask
from traffic_manager_msgs.srv import GetStationWaypoints

from behavior_manager.utils import format_pickup_tasks_log, format_station_waypoints_log, format_pose_log, format_feedback_log

import asyncio

""" ================================================================

< Action Server >
- service type : DispatchDeliveryTask
- service name : "dispatch_delivery_task"
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

$ ros2 action send_goal /delibot_1/dispatch_delivery_task task_manager_msgs/action/DispatchDeliveryTask "{pickups: [{station: 'fresh_station', handler: 'delibot_1', payload: [{sku: 'apple', quantity: 5}]}]}" --feedback

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
        self.waypoint_client = self.create_client(
            GetStationWaypoints, f"{self.robot_id}/get_station_waypoints")

        # Initialize topic publisher=========================
        self.pose_publisher = self.create_publisher(
            PoseStamped, "goal_pose", 10
        )
        self.get_logger().info(f"/goal_pose Topic publisher is ready!")



    async def handle_dispatch_task(self, goal_handle):
        """
        Handle incoming dispatch delivery task requests.
        """
        self.goal_handle = goal_handle
        pickups = goal_handle.request.pickups
        goal_log_message = format_pickup_tasks_log(pickups)
        self.get_logger().info(f"/{self.robot_id}_dispatch_delivery_task Goal received: \n{goal_log_message}")

        wp_result = await self.request_station_waypoints(pickups)
        if not wp_result:
            result = self.send_result(
                goal_handle, success=False, error_code=1, error_msg="Failed to get station waypoints."
            )
            return result
        
        self.publish_goal_pose(wp_result.station_waypoints[0])

        current_pose, distance_remaining = self.handle_pose()
        await self.send_feedback(current_pose, distance_remaining)

        result = self.send_result(
            goal_handle, success=True, error_code=0, error_msg="All tasks completed successfully."
        )
        return result


    async def request_station_waypoints(self, pickups):
        """
        Request waypoints for pickup stations from the Traffic Manager.
        """
        # Wait for the service server to be available
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'/{self.robot_id}/get_station_waypoints Waiting for service...')
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Service is available!")

        request = GetStationWaypoints.Request()
        request.pickups = pickups
        future = self.waypoint_client.call_async(request)
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Request sending...")

        result = await future

        if result is None or not result.station_waypoints:
            self.get_logger().error(f"/{self.robot_id}/get_station_waypoints Failed to receive waypoints.")
            return None
        
        log_message = format_station_waypoints_log(result.station_waypoints)
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Response received: \n{log_message}")

        return result

    def publish_goal_pose(self, station_waypoint):
        """
        Publish goal pose based on the received waypoint.
        """
        waypoint = station_waypoint.waypoint

        position = waypoint.pose.position
        orientation = waypoint.pose.orientation

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = position.x
        pose_msg.pose.position.y = position.y
        pose_msg.pose.position.z = position.z
        pose_msg.pose.orientation = orientation

        self.pose_publisher.publish(pose_msg)
        log_message = format_pose_log(pose_msg)
        self.get_logger().info(f"Published /goal_pose: \n{log_message}")


    async def send_feedback(self, current_pose, distance_remaining):
        """
        Send action feedbacks.
        """
        if hasattr(self, "goal_handle") and self.goal_handle:
            feedback_msg = DispatchDeliveryTask.Feedback()
            feedback_msg.current_pose = current_pose    # /amcl_pose or /tf
            feedback_msg.distance_remaining = distance_remaining  # Update with real distance
            self.goal_handle.publish_feedback(feedback_msg)

            log_message = format_feedback_log(current_pose, distance_remaining)
            self.get_logger().info(f"Feedback sent: \n{log_message}")
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
            self.get_logger().info(f"Task completed successfully: \n{log_message}")
        else:
            self.get_logger().error(f"Task failed: \n{log_message}")

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

    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()