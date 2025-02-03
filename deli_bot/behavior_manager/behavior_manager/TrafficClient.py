import rclpy
from rclpy.node import Node

from traffic_manager_msgs.srv import GetStationWaypoints

from traffic_manager.utils import format_pickup_tasks_log, format_station_waypoints_log

import asyncio
import threading

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
- {station: 'apple_shelf', handler: 'delibot_1', payload: [{sku: 'apple', quantity: 5}]}
- {station: 'peach_shelf', handler: 'delibot_1', payload: [{sku: 'peach', quantity: 3}]}"

================================================================ """

class TrafficClient(Node):
    def __init__(self, robot_id):
        self.robot_id = robot_id
        super().__init__(f"{self.robot_id}_traffic_client")

        # Initialize service server
        self.task_service = self.create_service(
            GetStationWaypoints, f"/{self.robot_id}/get_task_station", self.handle_task_station
        )
        self.get_logger().info(f"/{self.robot_id}/get_task_station Service is ready!")

        # Initialize service client
        self.traffic_client = self.create_client(
            GetStationWaypoints, f"/{self.robot_id}/get_station_waypoints")

        # Wait for the service server to be available
        while not self.traffic_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'/{self.robot_id}/get_station_waypoints Waiting for service...')
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Service is available!")

    def handle_task_station(self, request, response):
        loop = asyncio.get_event_loop()
        if loop.is_running():
            asyncio.create_task(self.async_handle_task_station(request, response))
        else:
            self.get_logger().warn("No running event loop found! Creating a new one...")
            thread = threading.Thread(target=self.run_async_task, args=(request, response))
            thread.start()

        return response
    
    def run_async_task(self, request, response):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.async_handle_task_station(request, response))
        
    async def async_handle_task_station(self, request, response):
        task = request.pickups
        log_message = format_pickup_tasks_log(self.robot_id, task)
        self.get_logger().info(f"/{self.robot_id}/get_task_station Request received: \n{log_message}")

        station_waypoint, log_message = await self.request_station_waypoints(task)
        if station_waypoint:
            response.station_waypoints = station_waypoint
            self.get_logger().info(f"/{self.robot_id}/get_task_station Response sent: \n{log_message}")
            if response.station_waypoints:
                self.get_logger().info(f"/{self.robot_id}/get_task_station Response Success!")
            else:
                self.get_logger().error(f"/{self.robot_id}/get_task_station Response is empty!")
            
        else:
            self.get_logger().error(f"/{self.robot_id}/get_task_station Failed to get station waypoints")

        await asyncio.sleep(0.1)


    async def request_station_waypoints(self, pickups):
        # Initialize a request
        request = GetStationWaypoints.Request()
        request.pickups = pickups
        future = self.traffic_client.call_async(request)
        self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Request sending...")

        try:
            result = await future
            if not result:
                self.get_logger().error(f"/{self.robot_id}/get_station_waypoints Failed to receive waypoints.")
                return None

            log_message = format_station_waypoints_log(self.robot_id, result.station_waypoints)
            self.get_logger().info(f"/{self.robot_id}/get_station_waypoints Response received: \n{log_message}")
            return result.station_waypoints, log_message
        
        except Exception as e:
            self.get_logger().error(f"/{self.robot_id}/get_station_waypoints Error: {str(e)}")
            return None
    

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