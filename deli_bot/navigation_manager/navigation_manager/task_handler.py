import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from task_manager_msgs.action import DispatchDeliveryTask
from traffic_manager_msgs.srv import GetStationWaypoints
from traffic_manager_msgs.action import SetTargetPose

from navigation_manager.utils import format_pickup_tasks_log, format_station_waypoints_log, format_pose_log, format_feedback_log

import asyncio
import time

"""
Description:
Receive the task and request waypoints.

---
Action Server:
    Action: DispatchDeliveryTask
    Server name: /{robot_id}/dispatch_delivery_task
---
Service Client:
    Service: GetStationWaypoints
    Client name: /{robot_id}/get_station_waypoints
---
Action Client:
    Action: SetTargetPose
    Client name: /{robot_id}/set_target_pose

---
Test Command:
$ ros2 action send_goal /delibot_1/dispatch_delivery_task task_manager_msgs/action/DispatchDeliveryTask "{pickups: [{station: '일반', handler: 'delibot_1', payload: [{sku: 'apple', quantity: 5}]}]}" --feedback

"""


class TaskHandler(Node):
    def __init__(self, robot_id: str):
        super().__init__(f"{robot_id}_task_handler")
        self.robot_id = robot_id

        # Initialize action server
        self.delivery_task_server = ActionServer(
            self, DispatchDeliveryTask, f"{self.robot_id}/dispatch_delivery_task", self.handle_dispatch_task)

        # Initialize service client
        self.waypoint_client = self.create_client(
            GetStationWaypoints, f"{self.robot_id}/get_station_waypoints")
        
        # Initialize action client
        self.target_pose_client = ActionClient(
            self, SetTargetPose, f"{self.robot_id}/set_target_pose")

        self.get_logger().info(f"Task Handler initialized.")

    async def handle_dispatch_task(self, goal_handle):
        """Handle incoming dispatch delivery task requests."""
        # Receive goal from TaskManager
        self.goal_handle = goal_handle
        pickups = goal_handle.request.pickups
        task_goal_log_message = format_pickup_tasks_log(pickups)
        self.get_logger().info(f"/{self.robot_id}_dispatch_delivery_task Goal received: \n{task_goal_log_message}")

        # Send request to TrafficManager and wait response
        wp_result = await self.request_station_waypoints(pickups)
        if not wp_result:
            result = self.send_result(
                goal_handle, success=False, error_code=1, error_msg="Failed to get station waypoints."
            )
            return result

        # Receive response from TrafficManager 
        # target_pose = wp_result.station_waypoints[0].waypoint
        # goal_msg = SetTargetPose.Goal()
        # goal_msg.target_pose = PoseStamped()
        # goal_msg.target_pose.header.frame_id = target_pose.header.frame_id
        # goal_msg.target_pose.pose = target_pose.pose

        # pose_goal_log_message = format_pose_log(goal_msg.target_pose)
        # self.get_logger().info(f"/set_target_pose Sending goal: \n{pose_goal_log_message}")

        target_pose = wp_result.station_waypoints
        goal_msg = SetTargetPose.target_poses

        # Send goal to BehaviorManager and receive feedback 
        tp_goal_handle = await self.target_pose_client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: self.feedback_callback(goal_handle, fb))
        
        if not tp_goal_handle.accepted:
            self.get_logger().warn("/set_target_pose Goal was rejected.")
            goal_handle.abort()
            # return SetTargetPose.Result(success=False)
            return DispatchDeliveryTask.Result(success=False)
        
        self.get_logger().info("/set_target_pose Goal accepted by BehaviorManager. Waiting for result...")

        # Receive result from BehaviorManager
        result_future = tp_goal_handle.get_result_async()
        result = await result_future

        tp_result = result.result

        # Return result to TaskManager
        if  tp_result.success and tp_result.error_code == 0:
            self.get_logger().info(f"/navigate_to_pose Goal reached successfully!: {tp_result.error_code}")
            goal_handle.succeed()
            return DispatchDeliveryTask.Result(
                success=False, error_code=0, error_msg="Task Succeed"
            )
        else:
            self.get_logger().error(f"/navigate_to_pose Failed to reach the goal : {tp_result.error_code} {tp_result.error_msg}")
            goal_handle.abort()
            return DispatchDeliveryTask.Result(
                success=False, error_code=3, error_msg=tp_result.error_msg)


    async def request_station_waypoints(self, pickups):
        """Request station waypoint"""
        # Wait for the service server to be available
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'/{self.robot_id}/get_station_waypoints Waiting for service...')
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Service is available!")
        
        # Initialize request
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

    def feedback_callback(self, goal_handle, feedback_msg):
        """Send feedback to TaskManager, which is received from BehaviorManager."""

        # Initialize feedback
        feedback = DispatchDeliveryTask.Feedback()
        feedback.current_pose = feedback_msg.feedback.current_pose
        feedback.distance_remaining = feedback_msg.feedback.distance_remaining

        # Send feedback to TaskManager
        goal_handle.publish_feedback(feedback)
        log_message = format_feedback_log(feedback.current_pose, feedback.distance_remaining)
        self.get_logger().info(f"/dispatch_delivery_task Feedback: \n{log_message}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskHandler("delibot_1")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(f"Task Handler stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()