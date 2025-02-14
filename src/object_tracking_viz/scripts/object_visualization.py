#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import json
import cv2
import threading


class ImageSubscriber(Node):
    """ Subscribes to /camera/image_raw and stores the latest frame """

    def __init__(self, shared_data, lock):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.br = CvBridge()
        self.shared_data = shared_data
        self.lock = lock

    def image_callback(self, data):
        """ Stores the latest image in shared memory """
        frame = self.br.imgmsg_to_cv2(data, "bgr8")
        with self.lock:
            self.shared_data["frame"] = frame
        self.get_logger().info("Updated frame.")


class ObjectDetection(Node):
    """ Subscribes to /object_detection and stores the latest detections """

    def __init__(self, shared_data, lock):
        super().__init__('object_detection_subscriber')
        self.subscription = self.create_subscription(
            String, '/object_detection', self.detection_callback, 10)
        self.shared_data = shared_data
        self.lock = lock

    def detection_callback(self, msg):
        """ Stores latest detections in shared memory """
        detections = json.loads(msg.data)
        with self.lock:
            self.shared_data["detections"] = detections
        self.get_logger().info(f"Updated detections: {len(detections)}")


class ObjectTracking(Node):
    """ Subscribes to /object_tracking and overlays tracking data on images """

    def __init__(self, shared_data, lock):
        super().__init__('object_tracking_subscriber')

        self.subscription = self.create_subscription(String, '/object_tracking', self.tracking_callback, 10)
        self.result_pub = self.create_publisher(Image, '/visualization/image', 10)
        self.br = CvBridge()
        
        self.shared_data = shared_data
        self.lock = lock

        # Timer to update visualization independently
        self.timer = self.create_timer(0.1, self.overlay_visualization)

        # Start a separate thread for cv2.imshow()
        self.display_thread = threading.Thread(target=self.display_images, daemon=True)
        self.display_thread.start()

    def tracking_callback(self, msg):
        """ Stores the latest tracking data in shared memory """
        tracked_objects = json.loads(msg.data)
        with self.lock:
            self.shared_data["tracked_objects"] = tracked_objects
        self.get_logger().info(f"Updated tracking objects: {len(tracked_objects)}")

    def overlay_visualization(self):
        """ Draws bounding boxes and tracking information on the latest image """
        with self.lock:
            frame = self.shared_data.get("frame", None)
            detections = self.shared_data.get("detections", [])
            tracked_objects = self.shared_data.get("tracked_objects", {})

        if frame is None:
            return  # No new frame yet, skip processing

        frame = frame.copy()

        # Draw object detections
        for detection in detections:
            x1, y1, x2, y2 = detection["bbox"]
            label = f"{detection['label']} ({detection['confidence']:.2f})"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw object tracking
        for object_id, centroid in tracked_objects.items():
            x, y = centroid
            cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID {object_id}", (int(x) + 10, int(y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Convert and publish processed image
        final_img = self.br.cv2_to_imgmsg(frame, "bgr8")
        self.result_pub.publish(final_img)

        # Store frame for separate display thread
        with self.lock:
            self.shared_data["visualized_frame"] = frame

    def display_images(self):
        """ Continuously displays images without blocking ROS execution """
        while True:
            with self.lock:
                frame = self.shared_data.get("visualized_frame", None)

            if frame is not None:
                cv2.imshow("Visualization", frame)
                cv2.waitKey(1)  # Keep the window active


def main(args=None):
    rclpy.init(args=args)

    # Shared memory & lock for synchronization
    shared_data = {"frame": None, "detections": [], "tracked_objects": {}, "visualized_frame": None}
    lock = threading.Lock()

    # Initialize nodes
    image_node = ImageSubscriber(shared_data, lock)
    detection_node = ObjectDetection(shared_data, lock)
    tracking_node = ObjectTracking(shared_data, lock)

    # Multi-threaded execution
    executor = MultiThreadedExecutor()
    executor.add_node(image_node)
    executor.add_node(detection_node)
    executor.add_node(tracking_node)

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
