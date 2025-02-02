import os
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from traffic_manager_msgs.srv import GetStationWaypoints
from traffic_manager_msgs.msg import StationWaypoint

from traffic_manager.utils import format_pickup_tasks_log, format_station_waypoints_log

""" ================================================================

< Service Server >
- service type : GetStationWaypoints
- service name : "get_station_waypoints"
- callback : handle_station_waypoints


< GetStationWaypoints >
# Request
task_manager_msgs/PickUp[] pickups

---
# Response
StationWaypoint[] station_waypoints


< Test Command >
$ ros2 service call /delibot_1/get_station_waypoints traffic_manager_msgs/srv/GetStationWaypoints "pickups:
- {station: 'apple_shelf', handler: 'delibot_1', payload: [{sku: 'apple', quantity: 5}]}
- {station: 'peach_shelf', handler: 'delibot_1', payload: [{sku: 'peach', quantity: 3}]}"


$ ros2 service call /delibot_2/get_station_waypoints traffic_manager_msgs/srv/GetStationWaypoints "pickups:
- {station: 'apple_shelf', handler: 'delibot_2', payload: [{sku: 'apple', quantity: 2}]}
- {station: 'peach_shelf', handler: 'delibot_2', payload: [{sku: 'peach', quantity: 4}]}"

================================================================ """

class TrafficManager(Node):
    def __init__(self):
        super().__init__('traffic_manager')

        # Load waypoints from YAML
        pkg_share_dir = get_package_share_directory('traffic_manager')
        waypoints_file = os.path.join(pkg_share_dir, 'config', 'waypoints.yaml')
        with open(waypoints_file, 'r') as f:
            data = yaml.safe_load(f)

        # Parse YAML data into pose_map
        self.pose_map = {}
        for place_name, pose_data in data.items():
            pose = PoseStamped()
            pose.header.frame_id = 'map'  # Fixed frame for navigation
            pose.pose.position.x = pose_data['pose']['x']
            pose.pose.position.y = pose_data['pose']['y']
            pose.pose.position.z = pose_data['pose'].get('z', 0.0)  # Default z=0.0
            pose.pose.orientation.x = pose_data['pose']['orientation'].get('x', 0.0)
            pose.pose.orientation.y = pose_data['pose']['orientation'].get('y', 0.0)
            pose.pose.orientation.z = pose_data['pose']['orientation'].get('z', 0.0)
            pose.pose.orientation.w = pose_data['pose']['orientation'].get('w', 1.0)
            self.pose_map[place_name] = pose

        # Support multiple robots
        self.robots = ["delibot_1", "delibot_2", "delibot_3"]
        self.station_waypoint_server = {}
        for robot_id in self.robots:
            self.station_waypoint_server[robot_id] = self.create_service(
                GetStationWaypoints, f"/{robot_id}/get_station_waypoints", self.handle_station_waypoints)
            self.get_logger().info(f"/{robot_id}/get_station_waypoints Service is ready!")


    def handle_station_waypoints(self, request, response):
        robot_id = request.pickups[0].handler
        response.station_waypoints = []
        pickup_log = format_pickup_tasks_log(robot_id, request.pickups)
        self.get_logger().info(f"/{robot_id}/get_station_waypoints Request received: \n{pickup_log}")

        for pickup in request.pickups:
            station_waypoint = StationWaypoint()
            station_waypoint.station = pickup.station

            if pickup.station in self.pose_map:
                station_waypoint.waypoint = self.pose_map[pickup.station]
                
            else:
                station_waypoint.waypoint = PoseStamped()
                self.get_logger().warn(f"No waypoint found for station: {pickup.station}")

            response.station_waypoints.append(station_waypoint)

        log_message = format_station_waypoints_log(robot_id, response.station_waypoints)
        self.get_logger().info(f"/{pickup.handler}/get_station_waypoints Response sent : \n{log_message}")

        if not response.station_waypoints:
            self.get_logger().error(f"/{pickup.handler}/get_station_waypoints Response is empty!")
        else:
            self.get_logger().info(f"/{pickup.handler}/get_station_waypoints Response Success!")

        return response
    
        
def main(args=None):
    rclpy.init(args=args)
    node = TrafficManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
