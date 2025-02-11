import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import math

class AlignWithPath(Node):
    def __init__(self):
        super().__init__('align_with_path')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/amcl_pose', self.pose_callback, 10)
        
        self.current_yaw = 0.0
        self.target_yaw = None
        self.timer = self.create_timer(0.1, self.align_with_path)

    def pose_callback(self, msg):
        orientation = msg.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(orientation)
        
        self.get_logger().info(f"Current heading: {math.degrees(self.current_yaw):.2f}°")

    def path_callback(self, msg):
        if len(msg.poses) >= 2:
            start = msg.poses[0].pose.position
            next_point = msg.poses[1].pose.position
            self.target_yaw = math.atan2(next_point.y - start.y, next_point.x - start.x)

            self.get_logger().info(f"Target heading received: {math.degrees(self.target_yaw):.2f}°")

    def align_with_path(self):
        if self.target_yaw is None:
            return

        # Compute heading error
        yaw_error = self.target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # Normalize between -pi to pi

        # Calculate P-controller input to reduce heading error
        cmd = Twist()
        if abs(yaw_error) > 0.01:  # Allowable heading error
            cmd.angular.z = 0.5 * yaw_error  # Proportional control for smooth rotation
        else:
            cmd.angular.z = 0.0  # Stop rotation when aligned
            self.get_logger().info("Alignment complete. Robot is facing the target direction.")

        self.cmd_pub.publish(cmd)

    @staticmethod
    def quaternion_to_yaw(orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = AlignWithPath()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()