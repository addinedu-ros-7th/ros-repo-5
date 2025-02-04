import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from TrafficClient import TrafficClient
from TaskServer import TaskServer
from NavClient import NavClient

class BehaviorManager(Node):
    def __init__(self):
        self.robot_id = "robot_1"
        super().__init__(f"{self.robot_id}_behavior_manager")

        # Initialize modules
        self.task_server = TaskServer(self.robot_id)
        self.traffic_client = TrafficClient(self.robot_id)
        self.nav_client = NavClient(self.robot_id)


    def execute_task(self):
        """
        Main workflow for behavior manager.
        """
        # Get task with station list
        # TODO: Implement get_task in TaskServer
        task = self.get_task()

        for station in task.stations:
            # Get Waypoint for the station
            waypoint = self.traffic_client.get_waypoint(station)

            if waypoint is None:
                self.get_logger().error(f"Failed to fetch waypoint for station: {station}")
                self.task_server.send_result(
                    success=False,
                    error_code=1,
                    error_msg=f"Failed to fetch waypoint for station: {station}"
                )
                return

            # Navigate to Waypoint
            self.nav_client.navigate_to_waypoint(
                waypoint, feedback_callback=self.pass_feedback_to_task_server)

            self.task_server.send_result(success=True)
            
        self.task_server.destroy_node()
        self.traffic_client.destroy_node()
        self.nav_client.destroy_node()

    def pass_feedback_to_task_server(self, feedback_msg):
        """
        Pass NavClient feedback(current_pose. distance_remaining)
        to Task Server.
        """
        feedback = feedback_msg.feedback
        self.task_server.send_feedback(
            current_pose=feedback.current_pose,
            distance_remaining=feedback.distance_remaining
        )

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
