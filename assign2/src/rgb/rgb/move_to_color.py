import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MoveToColor(Node):
    def __init__(self):
        super().__init__('move_to_color_node')

        # Subscriber for blob details topic
        self.subscription = self.create_subscription(
            String,
            'blob_details',
            self.blob_callback,
            10
        )

        # Publisher for TurtleBot velocity commands
        self.velocity_publisher = self.create_publisher(Twist, 'c3_05/cmd_vel', 10)

        self.largest_red_blob = None

    def blob_callback(self, msg):
        blob_details = eval(msg.data)  # Convert string message to dictionary
        if blob_details['color'] == 'red':
            if self.largest_red_blob is None or blob_details['size'] > self.largest_red_blob['size']:
                self.largest_red_blob = blob_details

        if self.largest_red_blob:
            self.move_to_blob(self.largest_red_blob)

    def move_to_blob(self, blob):
        twist = Twist()

        # Proportional control for velocity
        error_x = blob['center'][0] - 320  # Assuming image width is 640
        error_y = blob['center'][1] - 240  # Assuming image height is 480

        # Proportional gain
        kx = 0.001
        ky = 0.001

        twist.linear.x = ky * error_y
        twist.angular.z = -kx * error_x

        self.velocity_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    node = MoveToColor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()