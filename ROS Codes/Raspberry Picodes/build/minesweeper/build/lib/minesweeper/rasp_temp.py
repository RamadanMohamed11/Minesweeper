import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import RPi.GPIO as GPIO

coil_input_pin = 22          
proximity_sensor_pin = 17    
alert_output_pin = 16        

class MineDetectionPublisher(Node):
    def __init__(self):
        super().__init__('mine_detection_publisher')
        self.publisher_ = self.create_publisher(String, 'mine_detection', 10)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(coil_input_pin, GPIO.IN)
        GPIO.setup(proximity_sensor_pin, GPIO.IN)
        GPIO.setup(alert_output_pin, GPIO.OUT)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('192.168.0.128', 65433)
        self.sock.connect(self.server_address)
        
        self.get_logger().info("Connected to socket at 192.168.1.14:65433")
        
        self.timer = self.create_timer(0.1, self.check_mine_status)

    # def check_mine_status(self):
    #     coil_detected = GPIO.input(coil_input_pin) == GPIO.HIGH
    #     proximity_detected = GPIO.input(proximity_sensor_pin) == GPIO.HIGH
        
    #     if coil_detected and proximity_detected:
    #         message = "surface mine detected"
    #     elif coil_detected and not proximity_detected:
    #         message = "underground mine detected"
    #     else:
    #         message = "no mine detected"
        
    #     GPIO.output(alert_output_pin, GPIO.HIGH if coil_detected else GPIO.LOW)
        
    #     msg = String()
    #     msg.data = message
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f"Published: {msg.data}")

    #     self.sock.sendall(message.encode())
    #     self.get_logger().info(f"Sent to socket: {message}")
    def check_mine_status(self):
        # coil_detected = GPIO.input(coil_input_pin) == GPIO.HIGH
        proximity_detected = GPIO.input(proximity_sensor_pin) == GPIO.HIGH
        
        if proximity_detected:
            message = "mine detected"
        else:
            message = "no mine detected"
        
        GPIO.output(alert_output_pin, GPIO.HIGH if proximity_detected else GPIO.LOW)
        
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
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
