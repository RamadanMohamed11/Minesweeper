import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import socket
import time

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(String, 'joystick_direction', 10)
        self.joystick_init()
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('192.168.0.132', 65432)
        
        try:
            self.sock.connect(self.server_address)
            self.get_logger().info(f"Connected to socket at {self.server_address}")
        except Exception as e:
            self.get_logger().error(f"Could not connect to socket: {e}")
            return

        self.timer = self.create_timer(0.1, self.publish_joystick_direction)

    def joystick_init(self):
        pygame.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().info("No joysticks detected.")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Joystick initialized: {self.joystick.get_name()}")

    def publish_joystick_direction(self):
        pygame.event.pump()

        x_axis = self.joystick.get_axis(0)
        y_axis = self.joystick.get_axis(1)
        threshold = 0.5
        direction = ""

        if y_axis < -threshold and x_axis < -threshold:
            direction = "Forward-Left"
        elif y_axis < -threshold and x_axis > threshold:
            direction = "Forward-Right"
        elif y_axis > threshold and x_axis < -threshold:
            direction = "Backward-Left"
        elif y_axis > threshold and x_axis > threshold:
            direction = "Backward-Right"
        elif y_axis < -threshold:
            direction = "Forward"
        elif y_axis > threshold:
            direction = "Backward"
        elif x_axis < -threshold:
            direction = "Left"
        elif x_axis > threshold:
            direction = "Right"
        else:
            direction = "Stop"

        msg = String()
        msg.data = direction
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

        self.sock.sendall(msg.data.encode())

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
