import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from py_trees.blackboard import Blackboard

class GoalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('goal_pose_subscriber')
        
        # /goal_pose 토픽 구독
        self.subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10
        )
        
        self.get_logger().info("Listening for /goal_pose messages...")

    def goal_pose_callback(self, msg):
        # Blackboard 인스턴스 가져오기
        blackboard = Blackboard()
        
        # Blackboard의 'goal' 키를 업데이트
        blackboard.set("goal", msg)

        self.get_logger().info(f"Updated Blackboard: goal -> {msg.pose}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
