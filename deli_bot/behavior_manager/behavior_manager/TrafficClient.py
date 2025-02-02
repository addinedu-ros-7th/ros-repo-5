import rclpy
from rclpy.node import Node

from traffic_manager_msgs.srv import GetStationWaypoints
from traffic_manager_msgs.msg import StationWaypoint
from task_manager_msgs.msg import PickUp, Payload

from traffic_manager.utils import format_pickup_tasks_log, format_station_waypoints_log

""" ================================================================

< Service Client >
- service type : GetStationWaypoints
- service name : "get_station_waypoints"
- callback : get_station_waypoint_callback


< GetStationWaypoints >
# Request
task_manager_msgs/PickUp[] pickups

---
# Response
StationWaypoint[] station_waypoints


< Test Command >
$ ros2 service call /delibot_1/get_task_station traffic_manager_msgs/srv/GetStationWaypoints "pickups:
- {station: 'apple_shelf', handler: 'robot_1', payload: [{sku: 'apple', quantity: 5}]}
- {station: 'peach_shelf', handler: 'robot_1', payload: [{sku: 'peach', quantity: 3}]}"

================================================================ """

class TrafficClient(Node):
    def __init__(self, robot_id):
        self.robot_id = robot_id
        super().__init__(f"{self.robot_id}_traffic_client")

        # Initialize service server
        # self.task_service = self.create_service(
        #     GetStationWaypoints, f"/{self.robot_id}/get_task_station", self.get_task_station_callback
        # )
        # self.get_logger().info(f"/{self.robot_id}/get_task_station Service is ready!")

        # Initialize service client
        self.traffic_client = self.create_client(
            GetStationWaypoints, f"/{self.robot_id}/get_station_waypoints")

        # Wait for the service server to be available
        while not self.traffic_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'/{self.robot_id}/get_station_waypoints Waiting for service...')
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Service is available!")

        self.handle_task_station(robot_id)


    def handle_task_station(self, robot_id): #request, response):
        # TODO: Implement task server in TaskServer
        test_task = [
            PickUp(station='apple_shelf', handler=robot_id, payload=[Payload(sku='apple', quantity=5)]),
            PickUp(station='peach_shelf', handler=robot_id, payload=[Payload(sku='peach', quantity=3)]),
        ]
        log_message = format_pickup_tasks_log(self.robot_id, test_task)
        self.get_logger().info(f"/{self.robot_id}/get_task_station Request received: \n{log_message}")
        waypoint = self.request_station_waypoints(test_task)

    #     self.get_logger().info(f"/{self.robot_id}/get_task_station Request received: {len(request.pickups)} tasks")

    #     station_waypoints = self.get_station_waypoints(request.pickups)
    #     if station_waypoints:
    #         response.station_waypoints = station_waypoints
    #         self.get_logger().info(f"/{self.robot_id}/get_task_station Response sent: {response.station_waypoints}")
    #     else:
    #         self.get_logger().error(f"/{self.robot_id}/get_task_station Response failed: Failed to get waypoints")

    #     return response


    def request_station_waypoints(self, pickups):
        # Initialize a request
        request = GetStationWaypoints.Request()
        request.pickups = pickups
        future = self.traffic_client.call_async(request)
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Request sending...")
        
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if not result:
            self.get_logger().error(f"/{self.robot_id}/get_task_station Failed to receive waypoints.")
            return None
        
        log_message = format_station_waypoints_log(self.robot_id, result.station_waypoints)
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Response received: \n{log_message}")
        return result.station_waypoints


def main(args=None):
    rclpy.init(args=args)
    node = TrafficClient("delibot_1")

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass    
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()