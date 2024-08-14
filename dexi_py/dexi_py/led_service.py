import rclpy
from rclpy.node import Node
from led_msgs.srv import SetLEDPixel
from led_msgs.srv import SetLEDColor
import board
import neopixel

class LEDService(Node):

    def __init__(self):
        super().__init__('led_service')
        self.led_service = self.create_service(SetLEDPixel, 'set_led_pixel', self.set_led_pixel_callback)
        self.led_color_service = self.create_service(SetLEDColor, 'set_led_color', self.set_led_color_callback)
        self.pixel_pin = board.D12
        self.num_pixels = 10
        self.pixel_order = neopixel.GRB
        self.pixels = neopixel.NeoPixel(self.pixel_pin, self.num_pixels, brightness=0.2, auto_write=False, pixel_order=self.pixel_order)


    def set_led_pixel_callback(self, request, response):
        self.get_logger().info(str(request))
        self.pixels[request.index] = (request.r, request.g, request.b)
        self.pixels.show()
        response.success = True
        return response

    def set_led_color_callback(self, request, response):
        self.get_logger().info(str(request))
        response.success = True
        return response

    def rainbow(self):
        return

    def fill (self):
        return


def main(args=None):
    rclpy.init(args=args)
    led_service = LEDService()
    rclpy.spin(led_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()