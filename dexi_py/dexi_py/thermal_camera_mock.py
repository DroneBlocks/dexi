import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
from random import uniform
from colour import Color  # Install with `pip install colour`

class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')
        self.publisher_ = self.create_publisher(Image, 'thermal_image_rgb', 10)
        self.timer = self.create_timer(1.0, self.publish_thermal_image)

        # Sensor temperature range
        self.MINTEMP = 26.0
        self.MAXTEMP = 32.0
        self.COLORDEPTH = 1024

        # Create the color gradient from blue to red
        blue = Color("indigo")
        self.colors = list(blue.range_to(Color("red"), self.COLORDEPTH))
        self.colors = [
            (int(c.red * 255), int(c.green * 255), int(c.blue * 255)) for c in self.colors
        ]

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def generate_random_thermal_data(self):
        # Generate an 8x8 array with random values between MINTEMP and MAXTEMP
        return [uniform(self.MINTEMP, self.MAXTEMP) for _ in range(64)]

    def publish_thermal_image(self):
        # Generate random thermal data
        thermal_data = self.generate_random_thermal_data()

        # Map and constrain data to the color range
        pixels = [
            self.constrain(
                int(self.map_value(temp, self.MINTEMP, self.MAXTEMP, 0, self.COLORDEPTH - 1)),
                0,
                self.COLORDEPTH - 1,
            )
            for temp in thermal_data
        ]

        # Create RGB image
        rgb_data = []
        for pixel in pixels:
            rgb_data.extend(self.colors[pixel])  # Add RGB tuple as flat data

        # Create Image message
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'thermal_camera'
        msg.height = 8
        msg.width = 8
        msg.encoding = 'rgb8'  # RGB 8-bit per channel
        msg.is_bigendian = False
        msg.step = msg.width * 3  # 3 bytes per pixel (R, G, B)
        msg.data = rgb_data

        # Publish the image
        self.publisher_.publish(msg)
        self.get_logger().info('Published thermal RGB image with randomized data')

def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
