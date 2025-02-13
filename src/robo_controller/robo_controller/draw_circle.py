import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawNode(Node):
    def __init__(self):
        super().__init__(node_name="draw_circle")
        self.get_logger().info("Drawing circle...")
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.send_vel_command)

    def send_vel_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0

        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Initialise node
    node = DrawNode()
    rclpy.spin(node)

    rclpy.shutdown()
