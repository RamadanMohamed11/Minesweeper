import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class MineSubscriber(Node):
    def __init__(self):
        super().__init__('mine_mapping_subscriber')
        self.create_subscription(Int32, 'mine_x', self.x_callback, 10)
        self.create_subscription(Int32, 'mine_y', self.y_callback, 10)
        self.create_subscription(String, 'mine_type', self.type_callback, 10)

    def x_callback(self, msg):
        self.get_logger().info(f"Received x coordinate: {msg.data}")

    def y_callback(self, msg):
        self.get_logger().info(f"Received y coordinate: {msg.data}")

    def type_callback(self, msg):
        self.get_logger().info(f"Received mine type: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MineSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
