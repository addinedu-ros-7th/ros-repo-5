import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from task_manager_msgs.action import DispatchDeliveryTask
from traffic_manager_msgs.srv import GetStationWaypoints, SetTargetPose

from behavior_manager.utils import format_pickup_tasks_log, format_station_waypoints_log, format_pose_log

import asyncio
import time

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

        # Initialize action server
        self.delivery_task_server = ActionServer(
            self, DispatchDeliveryTask, f"{self.robot_id}/dispatch_delivery_task", self.handle_dispatch_task)
        self.get_logger().info(f"/{self.robot_id}/dispatch_delivery_task Action is ready!")

        # Initialize service client
        self.waypoint_client = self.create_client(
            GetStationWaypoints, f"{self.robot_id}/get_station_waypoints")
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Service is available!")

        # Initialize action client
        self.target_pose_client = self.ActionClient(
            self, SetTargetPose, f"{self.robot_id}/set_target_pose")
        self.get_logger().info(f"/{self.robot_id}/set_target_pose Action is available!")


    async def handle_dispatch_task(self, goal_handle):
        """
        Handle incoming dispatch delivery task requests.
        goal: pickups
        result: success
        feedback: current_pose, distance_remaining
        """
        # Receive goal from TaskManager
        self.goal_handle = goal_handle
        pickups = goal_handle.goal.pickups
        goal_log_message = format_pickup_tasks_log(pickups)
        self.get_logger().info(f"/{self.robot_id}_dispatch_delivery_task Goal received: \n{goal_log_message}")

        # Send request to TrafficManager and wait response
        wp_result = await self.request_station_waypoints(pickups)
        if not wp_result:
            result = self.send_result(
                goal_handle, success=False, error_code=1, error_msg="Failed to get station waypoints."
            )
            return result

        # Receive response from TrafficManager 
        target_pose = wp_result.station_waypoints[0]
        goal_msg = SetTargetPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = target_pose.header.frame_id
        goal_msg.pose.pose = target_pose.pose
        self.get_logger().info(f"/set_target_pose Sending goal: \n{goal_msg.pose}")

        # Send goal to BehaviorManager and receive feedback 
        # and return it to TaskServer 
        tp_goal_handle = await self.target_pose_client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: self.feedback_callback(goal_handle, fb))
        
        if not tp_goal_handle.accepted:
            self.get_logger().warn("/set_target_pose Goal was rejected.")
            goal_handle.abort()
            return SetTargetPose.Result(success=False)
        
        self.get_logger().info("/set_target_pose Goal accepted by BehaviorManager. Waiting for result...")

        # Receive result from BehaviorManager
        result_future = self.target_pose_client.get_result_async()
        result = await result_future

        # Return result fo TaskManager
        if result.status == 0:
            self.get_logger().info("/navigate_to_pose Goal reached successfully!")
            goal_handle.succeed()
            return SetTargetPose.Result(success=True)
        else:
            self.get_logger().warn("/navigate_to_pose Failed to reach the goal!")
            goal_handle.abort()
            return SetTargetPose.Result(success=False)


    async def request_station_waypoints(self, pickups):
        """
        Request: pickups
        Response: station_waypoints
        """
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
        """
        Send feedback to TaskManager, which is received from BehaviorManager.
        feedback: current_pose, distance_remaining
        """

        # Initialize feedback
        feedback = DispatchDeliveryTask.Feedback()
        feedback.current_pose = feedback_msg.current_pose
        feedback.distance_remaining = feedback_msg.distance_remaining

        # Send feedback to TaskManager
        goal_handle.publish_feedback(feedback)
        log_message = format_pose_log(feedback)
        self.get_logger().info(f"/dispatch_delivery_task Feedback: \n{log_message}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskServer("delibot_1")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()