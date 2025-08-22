import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__('joystick_subscriber')
        self.subscription = self.create_subscription(
            String,
            'joystick_direction',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received direction: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickSubscriber()
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
