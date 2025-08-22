import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String  # Import necessary message types
import random

class MinePublisher(Node):
    def __init__(self):
        super().__init__('mine_mapping_publisher')
        self.x_publisher_ = self.create_publisher(Int32, 'mine_x', 10)  # Publisher for x coordinate
        self.y_publisher_ = self.create_publisher(Int32, 'mine_y', 10)  # Publisher for y coordinate
        self.type_publisher_ = self.create_publisher(String, 'mine_type', 10)  # Publisher for mine type
        
        # Set a timer to publish coordinates every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_mine_data)

    def publish_mine_data(self):
        # Randomly generate mine data
        x = random.randint(1, 20)
        y = random.randint(1, 20)
        mine_type = random.choice(['Surface', 'Buried'])
        
        # Create messages for each value
        x_msg = Int32()
        x_msg.data = x

        y_msg = Int32()
        y_msg.data = y

        type_msg = String()
        type_msg.data = mine_type

        # Publish the messages to the respective topics
        self.x_publisher_.publish(x_msg)
        self.y_publisher_.publish(y_msg)
        self.type_publisher_.publish(type_msg)
        
        # Logging the published data
        self.get_logger().info(f"Published mine at ({x}, {y}): {mine_type}")

def main(args=None):
    rclpy.init(args=args)
    node = MinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(node.timer)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
