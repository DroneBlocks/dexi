"""
Python ROS2 service client to set DEXI LED pixels in a rainbow sequence
"""

import time
from dexi_interfaces.srv import LEDPixelColor
import rclpy
from rclpy.node import Node

class LEDPixelClient(Node):

    def __init__(self):
        super().__init__('led_pixel_client')
        self.client = self.create_client(LEDPixelColor, '/dexi/led_service/set_led_pixel_color')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, continue waiting...')
        self.request = LEDPixelColor.Request()

    def send_request(self, index, r, g, b):
        self.request.index = index
        self.request.r = r
        self.request.g = g
        self.request.b = b
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    led_pixel_client = LEDPixelClient()

    # Define ROYGBIV colors (Red, Orange, Yellow, Green, Blue, Indigo, Violet)
    colors = [
        (255, 0, 0),    # Red
        (255, 127, 0),  # Orange
        (255, 255, 0),  # Yellow
        (0, 255, 0),    # Green
        (0, 0, 255),    # Blue
        (75, 0, 130),   # Indigo
        (148, 0, 211)   # Violet
    ]

    num_pixels = 44
    num_colors = len(colors)

    # Loop through each pixel and set it to a color from the ROYGBIV sequence
    for i in range(num_pixels):
        # Determine the color for this pixel by cycling through the ROYGBIV colors
        color = colors[i % num_colors]
        r, g, b = color

        # Send the request to set the pixel color
        response = led_pixel_client.send_request(i, r, g, b)
        led_pixel_client.get_logger().info(f'Set pixel {i} to color {color}: {response}')

        # Add a delay of 100ms before setting the next pixel
        time.sleep(0.1)

    # Optionally turn off the LEDs after the animation
    for i in range(num_pixels):
        led_pixel_client.send_request(i, 0, 0, 0)
        time.sleep(0.05)

    led_pixel_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
