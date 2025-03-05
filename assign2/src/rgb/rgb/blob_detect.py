import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np

class BlobDetector(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.bridge = CvBridge()

        # Subscriber for image topic
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # You can change this to your camera topic
            self.image_callback,
            10
        )

        # Publisher for detected objects info
        self.object_publisher = self.create_publisher(String, 'object_info', 10)

        # Publisher for blob details
        self.blob_publisher = self.create_publisher(String, 'blob_details', 10)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Call function to detect red, green, blue objects
        objects = self.detect_objects(cv_image)

        # Draw the objects on the image
        for obj in objects:
            color = obj['color']
            center = obj['center']
            size = obj['size']
            # Draw a circle at the center of each detected object
            color_bgr = (0, 0, 255) if color == 'red' else (0, 255, 0) if color == 'green' else (255, 0, 0)
            cv2.circle(cv_image, center, 10, color_bgr, -1)
            cv2.putText(cv_image, f'{color}: {size}', (center[0] - 50, center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        # Show the image with detected objects
        # cv2.imshow("Object Detector", cv_image)
        # cv2.waitKey(1)  # Allow OpenCV to update the window

        # Publish the detected objects' info (color, center, size)
        object_info_msg = String()
        object_info_msg.data = str(objects)
        self.object_publisher.publish(object_info_msg)

        # Publish blob details (color, size, x, y)
        for obj in objects:
            blob_details_msg = String()
            blob_details_msg.data = f"Color: {obj['color']}, Size: {obj['size']}, X: {obj['center'][0]}, Y: {obj['center'][1]}"
            self.blob_publisher.publish(blob_details_msg)

    def detect_objects(self, image):
        # Convert the image to HSV for better color detection
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for Red, Green, and Blue
        color_ranges = {
            'red': ((0, 120, 70), (10, 255, 255)),   # Lower and upper HSV values for red
            'green': ((35, 100, 100), (85, 255, 255)), # Green color range
            'blue': ((100, 150, 0), (140, 255, 255))   # Blue color range
        }

        detected_objects = []

        # Iterate over each color range (red, green, blue)
        for color, (lower, upper) in color_ranges.items():
            # Create a mask for the color range
            mask = cv2.inRange(hsv_image, lower, upper)

            # Find contours in the masked image (blobs/objects)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Iterate over the contours and filter based on size
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Filter out small objects (threshold can be adjusted)
                    # Get the bounding box for each contour
                    x, y, w, h = cv2.boundingRect(contour)
                    center = (x + w // 2, y + h // 2)
                    size = cv2.contourArea(contour)
                    detected_objects.append({'color': color, 'center': center, 'size': size})

        # Sort the objects by color (red > green > blue) and then by size
        detected_objects.sort(key=lambda obj: (['red', 'green', 'blue'].index(obj['color']), -obj['size']))

        return detected_objects

def main(args=None):
    rclpy.init(args=args)

    node = BlobDetector()

    rclpy.spin(node)

    # Close the OpenCV window gracefully when the node is shut down
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

