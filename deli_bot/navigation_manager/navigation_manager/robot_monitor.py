import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32
from task_manager_msgs.msg import BatteryLevel


"""
Description:
Publish the battery state at a specific time interval.
---
Publishing Topics:
/{robot_id}/battery
    Type: task_manager_msgs/BatteryLevel/float32
---
Subscription Topic:
/pinky_battery_present
    Type: std_msgs/msg/Float32
"""

class RobotMonitor(Node):
    def __init__(self, robot_id):
        self.robot_id = robot_id
        super().__init__(f"{self.robot_id}_monitor")
        self.battery_sub = self.create_subscription(
            Float32, "/pinky_battery_present", self.battery_callback, 10
        )
        self.battery_pub = self.create_publisher(
            BatteryLevel, f"{self.robot_id}/battery", 10
        )
        self.get_logger().info(f"Robot Monitor initialized.")


    def battery_callback(self, msg):
        battery_msg = BatteryLevel()
        battery_msg.level = msg.data
        self.battery_pub.publish(battery_msg)
        # self.get_logger().info(f"/{self.robot_id}_adapter Battery State: {battery_msg.level}%")


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor("delibot_1")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Robot Monitor stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
