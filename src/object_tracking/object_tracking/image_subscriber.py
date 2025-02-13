import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageNode(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.get_logger().info("Subscribing Images...")

        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.br = CvBridge()
    
    def listener_callback(self, img_msg):
        self.get_logger().info('Receiving image...')
        current_frame = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
