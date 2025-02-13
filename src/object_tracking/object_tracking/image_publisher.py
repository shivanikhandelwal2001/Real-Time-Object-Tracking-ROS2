import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageNode(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.get_logger().info("Publishing Images...")

        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.br = CvBridge()

        self.img_count = 0
        self.timer = self.create_timer(0.02, self.timer_callback)
    
    def timer_callback(self):
        success, frame = self.camera.read()
        if success:
            frame = cv2.resize(frame, (820, 620), interpolation=cv2.INTER_CUBIC)
            ros_frame = self.br.cv2_to_imgmsg(frame, "bgr8")
            self.img_pub.publish(ros_frame)
            self.get_logger().info(f"Published image {self.img_count}")
            self.img_count += 1
        

def main(args=None):
    rclpy.init(args=args)
    node = ImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
