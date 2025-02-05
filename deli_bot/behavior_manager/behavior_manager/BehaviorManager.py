import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from nav2_msgs.action import NavigateToPose
from traffic_manager_msgs.srv import GetTargetPose

from traffic_manager.utils import format_pickup_tasks_log, format_station_waypoints_log

import asyncio

"""
task_server가 DispatchDeliveryTask goal을 받으면 nav_client에 주행 목표 전송.
nav_client가 Nav2에서 받은 피드백(현재 위치, 남은 거리)을 nav_feedback_relay 노드에 퍼블리시.
nav_feedback_relay가 데이터를 task_server에 전달하여 피드백을 갱신.
Nav2의 주행이 완료되면 task_server가 완료 처리.


< Test Command >
$ ros2 service call /delibot_1/get_target_pose traffic_manager_msgs/srv/GetTargetPose "{target_pose: {station: 'station_a', pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}}"

"""

class BehaviorManager(Node):
    def __init__(self, robot_id):
        self.robot_id = robot_id
        super().__init__(f"{self.robot_id}_behavior_manager")

        # Initialize service server
        self.pose_service = self.create_service(
            GetTargetPose, f"/{self.robot_id}/get_target_pose", self.handle_target_pose
        )
        self.get_logger().info(f"/{self.robot_id}/get_target_pose Service is ready!")

        # Initialize action client
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info(f"/navigate_to_pose Action is ready!")


    async def handle_target_pose(self, request, response):
        """
        Receive target pose from pose service
        """
        target_pose = request.target_pose
        if target_pose.pose is None:
            self.get_logger().error(f"/{self.robot_id}/get_target_pose Received empty pose, cannot send goal.")
            response.success = False
            return response
        
        goal_pose = target_pose.pose
        result = await self.send_goal(goal_pose)
        response.success = result
        self.get_logger().info(f"/{self.robot_id}/get_target_pose Response sent: {response.success}")
        return response


    async def send_goal(self, goal_pose):
        """
        Send a target pose to Nav2 as a action goal

        result
        0: success
        1: failure
        2: canceled
        """
        # Initialize action goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.get_logger().info(f"/navigate_to_pose Sending goal: \n{goal_msg.pose}")

        # Wait for the service server to be available
        await asyncio.sleep(0)
        if not self.nav_client.wait_for_server():
            self.get_logger().error("/navigate_to_pose Action server is not available!")
            return False
        self.get_logger().info("/navigate_to_pose Action is available!")

        # Send action goal to Nav2
        send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        goal_handle = await send_goal_future
        
        if not goal_handle.accepted:
            self.get_logger().warn("/navigate_to_pose Goal was rejected by Nav2.")
            return False
        
        # Receive action result from Nav2
        self.get_logger().info("/navigate_to_pose Goal accepted! Waiting for result...")
        result_future = goal_handle.get_result_async()
        result = await result_future
        
        # Return result
        if result.result == 0:
            self.get_logger().info("/navigate_to_pose Goal reached successfully!")
            return True
        else:
            self.get_logger().warn("/navigate_to_pose Failed to reach the goal!")
            return False


    def feedback_callback(self, feedback_msg):
        """
        Logging feedback form Nav2
        """
        self.get_logger().info(
            f"Navigation Progress: \n"
            f"X={feedback_msg.feedback.current_pose.pose.position.x}, "
            f"Y={feedback_msg.feedback.current_pose.pose.position.y}"
        )


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
