import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseNode(Node):
    def __init__(self):
        super().__init__(node_name="pose_subs")
        self.get_logger().info("Turtle pose info...")
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, msg: Pose):
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)

    # Initialise node
    node = PoseNode()
    rclpy.spin(node)

    rclpy.shutdown()
