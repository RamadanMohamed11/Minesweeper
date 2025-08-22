import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class MineDetectionSubscriber(Node):
    def __init__(self):
        
        super().__init__('mine_detection_subscriber')
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('', 65436)
        self.sock.bind(self.server_address)
        self.sock.listen(1)
        
        self.get_logger().info("Waiting for Raspberry Pi connection...")
        self.connection, self.client_address = self.sock.accept()
        self.get_logger().info(f"Connection established with {self.client_address}")
        
        self.timer = self.create_timer(0.1, self.receive_mine_status)

    def receive_mine_status(self):
        try:
            data = self.connection.recv(1024)
            if data:
                mine_status = data.decode().strip()
                if mine_status == "mine detected":
                    self.get_logger().info("Mine detected!")
                else:
                    self.get_logger().info("No mine detected.")
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")

    def destroy_node(self):
        super().destroy_node()
        self.connection.close()
        self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = MineDetectionSubscriber()
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
