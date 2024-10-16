"""
Python ROS2 topic subscriber to listen to DEXI GPIO pin state
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class GPIOSubscriber(Node):

    def __init__(self):
        super().__init__('gpio_subscriber')
        # Listen to GPIO 22 pin state
        self.pin = 22
        self.gpio_subscriber = self.create_subscription(Bool, '/dexi/gpio_input_{pin}'.format(pin=self.pin), self.gpio_callback, 10)

    def gpio_callback(self, msg):
        self.get_logger().info('Data for pin {pin} is: {data}'.format(pin=self.pin, data=msg.data))

def main(args=None):
    rclpy.init(args=args)
    gpio_subscriber = GPIOSubscriber()
    rclpy.spin(gpio_subscriber)
    gpio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()