import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

gripper_PWM_pin = 22
gripper_back_pin = 24

limit_switch1_pin = 32
limit_switch2_pin = 30

class GripperSubscriber(Node):
    def __init__(self):
        super().__init__('gripper_subscriber')
        self.subscription = self.create_subscription(String, 'gripper', self.listener_callback, 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(gripper_back_pin, GPIO.OUT)
        GPIO.setup(gripper_PWM_pin, GPIO.OUT)
        GPIO.setup(limit_switch1_pin, GPIO.IN)
        GPIO.setup(limit_switch2_pin, GPIO.IN)
    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        if msg.data != 'No mine detected':
            # Move the distance of the robot length, then make the gripper move down
            # ...
            while GPIO.input(limit_switch1_pin)==GPIO.HIGH:
                pass
            # Move the gripper up
            # ...
            while GPIO.input(limit_switch2_pin)==GPIO.HIGH:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = GripperSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()