import rclpy
from rclpy.node import Node


class CustomNode(Node):
    def __init__(self):
        super().__init__(node_name="hello_node")
        self.get_logger().info("Starting...")
        self.count = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Hello {self.count}")
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    # Initialise node
    node = CustomNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()