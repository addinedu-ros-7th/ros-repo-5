import os
import asyncio
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.timer import Timer

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from traffic_manager_msgs.action import SetTargetPose
from navigation_manager_msgs.msg import MotionCommand
from .motion_code_enums import MotionCode

from navigation_manager.utils import format_target_pose_log, format_feedback_log, format_pose_log


"""
Description:
Receive the task and request waypoints.

---
Action Server:
    msg type: SetTargetPose
    Action name: {robot_id}/set_target_pose

Subscription:
    msg type: MotionCommand
    topic name: motion_command


Action Client:
    msg type: NavigateToPose
    Action name: {robot_id}/navigate_to_pose

---
Test Command:
$ 
"""

class NavigationManager(Node):
    def __init__(self, robot_id):
        self.robot_id = robot_id
        super().__init__(f"{self.robot_id}_navigation_manager")

        # Initialize variables
        self.is_navigating = False
        self.current_goal_pose = None
        self.obstacle_data = None
        self.human_detected = False
        self.last_human_detection_time = 0.0

        self.target_pose_server = ActionServer(
            self, SetTargetPose, f"{self.robot_id}/set_target_pose", self.handle_target_pose
        )
        self.motion_subscription = self.create_subscription(
            MotionCommand, "motion_command", self.motion_cb, 10
        )
        self.nav2_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )

        self.nav_goal_handle = None

        self.get_logger().info("Navigation Manager initialized.")


    async def motion_cb(self, msg: MotionCommand):
        self.motion_command = msg.motion_code

        if self.motion_command == MotionCode.MOVE.value and self.awaiting_move:
            self.get_logger().info("MOVE command received while awaiting. Proceeding to next goal.")
            self.awaiting_move = False
            await self.send_next_goal()


    async def handle_target_pose(self, goal_handle):
        """Receive target pose from TaskServer."""

        target_pose = goal_handle.request.target_pose
   
        while not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info(f'/navigate_to_pose Waiting for server...')
        self.get_logger().info(f"/navigate_to_pose Service is available!")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = target_pose.header.frame_id
        goal_msg.pose.pose = target_pose.pose
        
        log_message = format_pose_log(goal_msg.pose)
        self.get_logger().info(f"/navigate_to_pose Sending goal: \n{log_message}")

        # Send goal to Nav2
        self.nav_goal_handle = await self.nav_client.send_goal_async(
            goal_msg, feedback_callback=lambda nav_fb: self.feedback_callback(goal_handle, nav_fb))

        if not self.nav_goal_handle.accepted:
            goal_handle.abort()
            self.get_logger().warn("/navigate_to_pose Goal was rejected.")
            return SetTargetPose.Result(success=False)

        self.get_logger().info("/navigate_to_pose Goal accepted by Nav2. Waiting for result...")

        result_future = self.nav_goal_handle.get_result_async()
        result = await result_future

        # Return result to TaskServer
        return self.result_callback(goal_handle, result)


    def feedback_callback(self, goal_handle, feedback_msg):
        """Send feedback to TaskServer from Nav2."""

        feedback = SetTargetPose.Feedback()
        feedback.current_pose = feedback_msg.feedback.current_pose
        feedback.distance_remaining = feedback_msg.feedback.distance_remaining

        # Send feedback to TaskServer
        goal_handle.publish_feedback(feedback)
        # log_message = format_feedback_log(feedback.current_pose, feedback.distance_remaining)
        # self.get_logger().info(f"/dispatch_delivery_task Feedback: \n{log_message}")


    def result_callback(self, goal_handle, result):
        nav_result = result.result
        if nav_result.error_code == 0:
            self.get_logger().info(f"Navigation succeeded!: {nav_result}")
            goal_handle.succeed()
            return SetTargetPose.Result(
                success=True, error_code=0, error_msg="Navigation succeeded!")
        else:
            self.get_logger().error(f"Navigation failed with error: {nav_result.error_code} {nav_result.error_msg}")
            goal_handle.abort()
            return SetTargetPose.Result(
                success=False, error_code=nav_result.error_code, error_msg=nav_result.error_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager("delibot_1")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation Manager stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
