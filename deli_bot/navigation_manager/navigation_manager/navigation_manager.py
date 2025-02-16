import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.timer import Timer

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from traffic_manager_msgs.action import SetTargetPose
from navigation_manager_msgs.msg import ObstacleDetected

from navigation_manager.utils import format_target_pose_log, format_feedback_log, format_pose_log

import asyncio
import time

"""
Description:
Receive the task and request waypoints.

---
Action Server:
    msg type: SetTargetPose
    Action name: /{robot_id}/set_target_pose
    Server name: target_pose_server
---
Topic Subscription:
    msg type: ObstacleDetected
    topic name: /obstacle
    Sub name: obstacle_sub
---
Action Client:
    msg type: NavigateToPose
    Action name: {robot_id}/navigate_to_pose
    Client name: nav_client

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

        # Initialize action server
        self.target_pose_server = ActionServer(
            self, SetTargetPose, f"{self.robot_id}/set_target_pose", self.handle_target_pose)
        # self.get_logger().info(f"/{self.robot_id}/set_target_pose Action is ready!")

        # Initialize topic subscription
        self.obstacle_sub = self.create_subscription(
            ObstacleDetected, 'obstacle', self.obstacle_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize publisher for pausing navigation
        self.pause_nav_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize action client for Nav2
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        # self.get_logger().info(f"/navigate_to_pose Action client is ready!")

        # Timer for checking human disappearance
        self.timer = self.create_timer(0.5, self.check_human_disappearance)

        # Initialize nav_goal_handle
        self.nav_goal_handle = None

        self.get_logger().info("Navigation Manager initialized.")


    def obstacle_callback(self, msg: ObstacleDetected):
        """
        Callback when obstacle information is received.
        """
        self.obstacle_data = msg

        obstacle_id = msg.class_id
        obstacle_name = msg.class_name
        # distance = self.estimate_distance(msg)
        bbox = msg.bbox
        
        if not self.is_navigating:
            return
        
        if obstacle_name == "HUMAN":
            current_time = time.time()

            # if msg.distance < 5.0:
            #     self.human_detected = True
            #     self.last_human_detection_time = current_time
            #     self.stop_robot()
            #     self.get_logger().warn("Human detected! Stopping the robot.")
            # elif not self.human_detected:
            #     self.resume_navigation()

            x_min = bbox.center.position.x - bbox.size.x / 2
            x_max = bbox.center.position.x + bbox.size.x /2
            y_min = bbox.center.position.y- bbox.size.y / 2
            y_max = bbox.center.position.y + bbox.size.y / 2

            if y_max < 200:
                self.human_detected = True
                self.last_human_detection_time = current_time
                self.stop_robot()
                self.get_logger().warn("Human detected! Stopping the robot.")

    def check_human_disappearance(self):
        """
        Check if the human has disappeared and resume navigation if needed.
        """
        current_time = time.time()

        if self.human_detected and (current_time - self.last_human_detection_time > 1.0):
            self.human_detected = False
            self.get_logger().info("No human detected. Resuming movement.")
            self.resume_navigation()

    def stop_robot(self):
        """
        Stop the robot's motion by publishing zero velocity.
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.is_navigating = False

    def resume_navigation(self):
        """
        Resume navigation by re-sending the previous goal if available.
        """
        if self.is_navigating:
            self.get_logger().warn("Robot is already navigating. No need to resume.")
            return
        if self.current_goal_pose is None:
            self.get_logger().warn("No previous navigation goal to resume.")
            return
        
        self.get_logger().info("Resending navigation goal to Nav2.")

        # 기존 목표를 다시 전송
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_goal_pose

        future = self.nav_client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("Failed to resume navigation. Goal was rejected.")
            else:
                self.get_logger().info("Navigation resumed successfully.")
                self.is_navigating = True  # 주행이 시작되었음을 표시

        future.add_done_callback(goal_response_callback)


    async def handle_target_pose(self, goal_handle):
        """
        Receive target pose from TaskServer.
        goal: target_pose
        response:
        feedback:
        """
        # Receive goal from TaskServer
        # self.goal_handle = goal_handle
        target_pose = goal_handle.request.target_pose
        self.is_navigating = True

        self.current_goal_pose = PoseStamped()
        self.current_goal_pose.header.frame_id = target_pose.header.frame_id
        self.current_goal_pose.pose = target_pose.pose

        pose_log_message = format_target_pose_log(target_pose)
        # self.get_logger().info(f"{self.robot_id}/set_target_pose Goal received: \n{pose_log_message}")
        self.get_logger().info(f"is navigating = {self.is_navigating}")
        """
        Send goal pose to Nav2.
        goal: goal_pose
        response:
        feedback:
        """
        # Wait for the service server to be available
        while not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info(f'/navigate_to_pose Waiting for server...')
        # self.get_logger().info(f"/navigate_to_pose Service is available!")
        
        # Initialize goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = target_pose.header.frame_id
        goal_msg.pose.pose = target_pose.pose
        
        log_message = format_pose_log(goal_msg.pose)
        self.get_logger().info(f"/navigate_to_pose Sending goal: \n{log_message}")

        # Send goal to Nav2 and receive feedback 
        # and return it to TaskServer 
        try:
            self.nav_goal_handle = await self.nav_client.send_goal_async(
                goal_msg, feedback_callback=lambda nav_fb: self.feedback_callback(goal_handle, nav_fb))

            if not self.nav_goal_handle.accepted:
                self.is_navigating = False
                goal_handle.abort()
                self.get_logger().warn("/navigate_to_pose Goal was rejected.")
                return SetTargetPose.Result(success=False)

            self.get_logger().info("/navigate_to_pose Goal accepted by Nav2. Waiting for result...")

            # Receive result from Nav2
            result_future = self.nav_goal_handle.get_result_async()
            result = await result_future
            self.is_navigating = False

            # Return result to TaskServer
            return self.result_callback(goal_handle, result)
        
        except Exception as e:
            self.get_logger().error(f"/navigate_to_pose error in sending goal: {str(e)}")
            self.is_navigating = False
            goal_handle.abort()
            return SetTargetPose.Result(success=False)

    def feedback_callback(self, goal_handle, feedback_msg):
        """
        Send feedback to TaskServer from Nav2.
        feedback: current_pose, distance_remaining
        """
        # Initialize feedback
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
        # elif result.result == 1:
        #     self.get_logger().info("Navigation was canceled.")
        #     goal_handle.abort()
        #     return SetTargetPose.Result(success=False)
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
