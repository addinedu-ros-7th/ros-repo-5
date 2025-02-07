import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from traffic_manager_msgs.action import SetTargetPose

from behavior_manager.utils import format_target_pose_log, format_feedback_log, format_pose_log

import asyncio

"""
task_server가 DispatchDeliveryTask goal을 받으면 nav_client에 주행 목표 전송.
nav_client가 Nav2에서 받은 피드백(현재 위치, 남은 거리)을 nav_feedback_relay 노드에 퍼블리시.
nav_feedback_relay가 데이터를 task_server에 전달하여 피드백을 갱신.
Nav2의 주행이 완료되면 task_server가 완료 처리.


< Test Command >
$ ros2 service call /delibot_1/set_target_pose traffic_manager_msgs/srv/SetTargetPose "{target_pose: {station: 'station_a', pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}}"

"""

class BehaviorManager(Node):
    def __init__(self, robot_id):
        self.robot_id = robot_id
        super().__init__(f"{self.robot_id}_behavior_manager")

        # Initialize action server
        self.target_pose_server = ActionServer(
            self, SetTargetPose, f"{self.robot_id}/set_target_pose", self.handle_target_pose)
        self.get_logger().info(f"/{self.robot_id}/set_target_pose Action is ready!")

        # Initialize action client for Nav2
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info(f"/navigate_to_pose Action client is ready!")


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
        pose_log_message = format_target_pose_log(target_pose)
        self.get_logger().info(f"{self.robot_id}/set_target_pose Goal received: \n{pose_log_message}")

        """
        Send goal pose to Nav2.
        goal: goal_pose
        response:
        feedback:
        """
        # Wait for the service server to be available
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'/navigate_to_pose Waiting for server...')
        self.get_logger().info(f"/navigate_to_pose Service is available!")
        
        # Initialize goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = target_pose.header.frame_id
        goal_msg.pose.pose = target_pose.pose
        
        log_message = format_pose_log(goal_msg.pose)
        self.get_logger().info(f"/navigate_to_pose Sending goal: \n{log_message}")

        # Send goal to Nav2 and receive feedback 
        # and return it to TaskServer 
        nav_goal_handle = await self.nav_client.send_goal_async(
            goal_msg, feedback_callback=lambda nav_fb: self.feedback_callback(goal_handle, nav_fb))

        if not nav_goal_handle.accepted:
            self.get_logger().warn("/navigate_to_pose Goal was rejected.")
            goal_handle.abort()
            return SetTargetPose.Result(success=False)

        self.get_logger().info("/navigate_to_pose Goal accepted by Nav2. Waiting for result...")
        
        # Receive result from Nav2
        result_future = nav_goal_handle.get_result_async()
        result = await result_future

        # Return result to TaskServer
        if result.status == 0:
            self.get_logger().info("/navigate_to_pose Goal reached successfully!")
            goal_handle.succeed()
            return SetTargetPose.Result(success=True)
        else:
            self.get_logger().warn("/navigate_to_pose Failed to reach the goal!")
            goal_handle.abort()
            return SetTargetPose.Result(success=False)


    def feedback_callback(self, goal_handle, feedback_msg):
        """
        Send feedback to TaskServer, which is received from Nav2.
        feedback: current_pose, distance_remaining
        """
        # Initialize feedback
        feedback = SetTargetPose.Feedback()
        feedback.current_pose = feedback_msg.feedback.current_pose
        feedback.distance_remaining = feedback_msg.feedback.distance_remaining

        # Send feedback to TaskServer
        goal_handle.publish_feedback(feedback)
        log_message = format_feedback_log(feedback.current_pose, feedback.distance_remaining)
        self.get_logger().info(f"/dispatch_delivery_task Feedback: \n{log_message}")


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager("delibot_1")
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
