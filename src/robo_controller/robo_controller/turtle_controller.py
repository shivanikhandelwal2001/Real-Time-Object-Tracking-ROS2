import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial


class TurtleNode(Node):
    def __init__(self):
        super().__init__(node_name="turtle_controller")
        self.get_logger().info("Turtle controller...")
        self.previous_x = 0
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_shivani", 10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose_shivani", self.pose_callback, 10)

    def pose_callback(self, msg: Pose):
        cmd = Twist()

        if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.8
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

        if msg.x > 5.5 and self.previous_x <= 5.5:
            self.get_logger().info("Set color to red...")
            self.call_set_pen_service(255, 0, 0, 3, 0)
            self.previous_x = msg.x
        elif msg.x <= 5.5 and self.previous_x > 5.5:
            self.get_logger().info("Set color to yellow...")
            self.call_set_pen_service(0, 255, 0, 3, 0)
            self.previous_x = msg.x

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = SetPen.Request()
        request.r, request.g, request.b, request.width, request.off = r, g, b, width, off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)

    # Initialise node
    node = TurtleNode()
    rclpy.spin(node)

    rclpy.shutdown()
