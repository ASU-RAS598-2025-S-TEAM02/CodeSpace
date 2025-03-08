import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from . import C3,RPI

class MoveToColor(Node):
    def __init__(self):
        super().__init__('move_to_color_node')

        # Subscriber for blob details topic
        self.subscription = self.create_subscription(
            String,
            f"{RPI}/blob_details",
            self.blob_callback,
            10
        )

        # Publisher for TurtleBot velocity commands
        out_topic = f"{C3}/cmd_vel"
        self.velocity_publisher = self.create_publisher(Twist, out_topic, 10)

        self.largest_red_blob = None

    def blob_callback(self, msg):
        # Parse the simplified blob format: "color,size,x,y;color,size,x,y;..."
        blob_details_str = msg.data
        
        # Skip processing if empty
        if not blob_details_str:
            return
        
        # Split by semicolons to get individual blob details
        blob_details_list = blob_details_str.split(';')
        
        # Reset largest red blob
        self.largest_red_blob = None
        
        # Process each blob
        for blob_detail in blob_details_list:
            # Skip empty entries
            if not blob_detail:
                continue
            
            # Split by comma to get components
            try:
                color, size, x, y = blob_detail.split(',')
                size = int(size)
                x = int(x)
                y = int(y)
                
                # Process red blobs only
                if color == 'red':
                    blob = {'color': color, 'size': size, 'x': x, 'y': y}
                    if self.largest_red_blob is None or blob['size'] > self.largest_red_blob['size']:
                        self.largest_red_blob = blob
            except ValueError:
                self.get_logger().error(f"Failed to parse blob detail: {blob_detail}")
        
        # If a red blob was found, move to it
        if self.largest_red_blob:
            self.move_to_blob(self.largest_red_blob)

    def move_to_blob(self, blob):
        twist = Twist()

        # Center the blob horizontally
        error_x = int(blob['x']) - 320  # Assuming image width is 640
        
        # Use size as proxy for distance (smaller = further away)
        size_factor = min(blob['size'] / 10000.0, 1.0)  # Normalize size
        
        # Set velocities
        twist.angular.z = -0.001 * error_x  # Turn to center the blob
        twist.linear.x = 0.2 * (1.0 - size_factor)  # Forward speed based on blob size
        
        self.velocity_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    node = MoveToColor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()