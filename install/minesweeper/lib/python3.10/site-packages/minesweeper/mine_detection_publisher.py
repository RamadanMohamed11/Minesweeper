import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class MineDetectionPublisher(Node):
    def __init__(self):
        super().__init__('mine_detection_publisher')
        self.publisher_ = self.create_publisher(String, 'mine_detection', 10)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('192.168.1.14', 65433)
        self.sock.connect(self.server_address)
        
        self.get_logger().info("Connected to socket at 192.168.1.14:65433")
        
        self.create_timer(0.1, self.check_mine_status_callback)
    


    def check_mine_status_callback(self):
        user_input = input("Press 1 for 'mine detected', 0 for 'no mine': ")
        if user_input == '1':
            message = "mine detected"
        elif user_input == '0':
            message = "no mine"
        else:
            self.get_logger().info("Invalid input, press 1 or 0.")
        
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

        self.sock.sendall(message.encode())
        self.get_logger().info(f"Sent to socket: {message}")

def main(args=None):
    rclpy.init(args=args)
    node = MineDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
