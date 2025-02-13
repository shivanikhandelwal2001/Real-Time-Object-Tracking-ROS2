#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
from ultralytics import YOLO
import json


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.get_logger().info("Subscribing Images...")

        self.model = YOLO('yolov8n.pt')

        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_pub = self.create_publisher(String, '/object_detection', 10)
        self.br = CvBridge()
    
    def image_callback(self, img_msg):
        self.get_logger().info('Received image...')

        frame = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        results = self.model(frame)

        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = result.names[int(box.cls[0])]
                confidence = float(box.conf[0])

                detections.append({
                    "label": label,
                    "confidence": confidence,
                    "bbox": [x1, y1, x2, y2]
                })

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        detection_msg = String()
        detection_msg.data = json.dumps(detections)
        self.detection_pub.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == "__main__":
    main()
