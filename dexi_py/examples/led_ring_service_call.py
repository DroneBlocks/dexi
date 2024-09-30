"""
Python ROS2 service client to set DEXI LED ring color
"""

import time
from dexi_interfaces.srv import LEDRingColor
import rclpy
from rclpy.node import Node

class LEDRingClient(Node):

    def __init__(self):
        super().__init__('led_ring_client')
        self.client = self.create_client(LEDRingColor, '/dexi/led_service/set_led_ring_color')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, continue waiting...')
        self.request = LEDRingColor.Request()

    def send_request(self, led_ring_color):
        self.request.color = led_ring_color
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    led_ring_client = LEDRingClient()

    # Set LED ring purple
    response = led_ring_client.send_request('purple')
    led_ring_client.get_logger().info(str(response))

    time.sleep(3)

    # Set LED ring white
    response = led_ring_client.send_request('white')
    led_ring_client.get_logger().info(str(response))

    led_ring_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()