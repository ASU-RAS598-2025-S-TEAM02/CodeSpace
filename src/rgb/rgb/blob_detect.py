import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
from . import CAMERA_TOPIC, RPI

class BlobDetector(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.bridge = CvBridge()

        # Subscriber for image topic
        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )

        # Publisher for blob details
        output_topic = f"{RPI}/blob_details"
        self.blob_publisher = self.create_publisher(String, output_topic, 50)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect objects
        objects = self.detect_objects(cv_image)
            
        # Simplified blob format: color,size,x,y;color,size,x,y;...
        blob_details = ""
        for obj in objects:
            blob_detail = f"{obj['color']},{obj['size']},{obj['center'][0]},{obj['center'][1]};"
            blob_details += blob_detail
            
        # Publish simplified blob details
        if blob_details:
            object_info_msg = String()
            object_info_msg.data = blob_details
            self.blob_publisher.publish(object_info_msg)

    def detect_objects(self, image):
        # Convert image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges
        color_ranges = {
            'red': ((0, 120, 70), (10, 255, 255)),   # Lower and upper HSV values for red
            'green': ((35, 100, 100), (85, 255, 255)), # Green color range
            'blue': ((100, 150, 0), (140, 255, 255))   # Blue color range
        }

        detected_objects = []

        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv_image, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Ignore small blobs
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        size = cv2.contourArea(contour)
                        detected_objects.append({'color': color, 'center': center, 'size': size})

        # Sort by color (red > green > blue) and then by size (largest first)
        detected_objects.sort(key=lambda obj: (['red', 'green', 'blue'].index(obj['color']), -obj['size']))

        return detected_objects

def main(args=None):
    time.sleep(2)  # Optional delay

    rclpy.init(args=args)
    node = BlobDetector()
    rclpy.spin(node)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
