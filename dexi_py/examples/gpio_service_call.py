"""
Python ROS2 service client to write DEXI GPIO output high and low
"""

import time
import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import GPIOSend

class GPIOServiceClient(Node):

    def __init__(self):
        super().__init__('gpio_service_client')
        # Toggle GPIO pin 21
        self.pin = 21
        self.client = self.create_client(GPIOSend, '/dexi/gpio_writer_service/write_gpio_{pin}'.format(pin=self.pin))
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, continue waiting...')
        self.request = GPIOSend.Request()

    def send_request(self, pin_state):
        self.request.pin = self.pin
        self.request.state = pin_state
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    gpio_service_client = GPIOServiceClient()

    # Send high value to GPIO pin
    response = gpio_service_client.send_request(True)
    gpio_service_client.get_logger().info(str(response))

    time.sleep(3)

    # Send low value to GPIO pin
    response = gpio_service_client.send_request(False)
    gpio_service_client.get_logger().info(str(response))

    gpio_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()