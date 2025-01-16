import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
from scipy.interpolate import griddata
from colour import Color
import busio
import board
import adafruit_amg88xx

class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')
        self.publisher_ = self.create_publisher(Image, 'thermal_image_rgb', 10)
        self.timer = self.create_timer(1.0, self.publish_thermal_image)

        # Initialize the AMG88XX sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(i2c)

        # Sensor configuration
        self.MINTEMP = 26.0
        self.MAXTEMP = 32.0
        self.COLORDEPTH = 1024
        self.grid_x, self.grid_y = np.mgrid[0:7:32j, 0:7:32j]
        self.points = [(ix // 8, ix % 8) for ix in range(64)]

        # Generate color gradient
        blue = Color("indigo")
        self.colors = list(blue.range_to(Color("red"), self.COLORDEPTH))
        self.colors = [
            (int(c.red * 255), int(c.green * 255), int(c.blue * 255)) for c in self.colors
        ]

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def map_value(self, x, in_min, in_max, out_min, out_max):
        if in_max == in_min:  # Avoid division by zero
            return (out_min + out_max) / 2  # Map to the middle of the range
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def get_camera_data(self):
        # Fetch raw thermal data from the sensor
        thermal_data = []
        for row in self.amg.pixels:
            thermal_data.extend(row)
        return thermal_data

    def publish_thermal_image(self):
        # Fetch the thermal data
        thermal_data = self.get_camera_data()

        # Dynamically adjust MINTEMP and MAXTEMP for better contrast
        current_min = min(thermal_data)
        current_max = max(thermal_data)
        dynamic_min = max(self.MINTEMP, current_min)
        dynamic_max = min(self.MAXTEMP, current_max)

        # Map data to the color range
        mapped_pixels = [
            self.map_value(
                temp, dynamic_min, dynamic_max, 0, self.COLORDEPTH - 1
            ) for temp in thermal_data
        ]

        # Perform bicubic interpolation for smooth rendering
        interpolated = griddata(self.points, mapped_pixels, (self.grid_x, self.grid_y), method="cubic")

        # Flatten the interpolated array into RGB data
        rgb_data = []
        for row in interpolated:
            for pixel in row:
                rgb_data.extend(self.colors[self.constrain(int(pixel), 0, self.COLORDEPTH - 1)])

        # Create and publish the ROS2 Image message
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'thermal_camera'
        msg.height = 32  # Interpolated height
        msg.width = 32  # Interpolated width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = msg.width * 3  # 3 bytes per pixel
        msg.data = rgb_data

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published thermal image: Min Temp={dynamic_min:.2f}, Max Temp={dynamic_max:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
