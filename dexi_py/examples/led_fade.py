"""
Python ROS2 service client to set DEXI LED pixels in a rainbow chase sequence with ultra-fast color fade effect
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
        self.request.r = int(r)
        self.request.g = int(g)
        self.request.b = int(b)
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def fade_between_colors(self, index, start_color, end_color, steps, delay=0.002):
        """Fade very quickly from start_color to end_color in minimal steps."""
        for step in range(steps):
            r = start_color[0] + (end_color[0] - start_color[0]) * step / steps
            g = start_color[1] + (end_color[1] - start_color[1]) * step / steps
            b = start_color[2] + (end_color[2] - start_color[2]) * step / steps
            self.send_request(index, r, g, b)
            time.sleep(delay)  # Even shorter delay for ultra-fast fading

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
    middle_index = num_pixels // 2
    fade_steps = 5  # Minimal steps for a very fast fade
    fade_delay = 0.002  # Ultra-short delay for extremely rapid transition

    # Run the animation through each color with ultra-fast fading
    while True: 
        for i in range(len(colors)):
            start_color = colors[i]
            end_color = colors[(i + 1) % len(colors)]  # Loop to the first color after the last

            led_pixel_client.get_logger().info(f'Starting ultra-fast chase with color fade from {start_color} to {end_color}')

            # Chase from both ends to the middle with an ultra-fast fade effect
            for j in range(middle_index):
                led_pixel_client.fade_between_colors(j, start_color, end_color, fade_steps, fade_delay)
                led_pixel_client.fade_between_colors(num_pixels - j - 1, start_color, end_color, fade_steps, fade_delay)

            # Pause briefly after meeting in the middle
            time.sleep(0.1)

            # Chase off (fade to off) from the middle to the ends with an ultra-fast fade
            for j in range(middle_index):
                led_pixel_client.fade_between_colors(middle_index - j - 1, end_color, (0, 0, 0), fade_steps, fade_delay)
                led_pixel_client.fade_between_colors(middle_index + j, end_color, (0, 0, 0), fade_steps, fade_delay)

    led_pixel_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()