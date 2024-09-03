import rclpy
from rclpy.node import Node
from dexi_interfaces.srv import LEDPixelColor
from dexi_interfaces.srv import LEDRingColor
import board
import neopixel
from enum import Enum

from adafruit_led_animation.animation.solid import Solid
from adafruit_led_animation.color import (
    RED as red,
    YELLOW as yellow,
    ORANGE as orange,
    GREEN as green,
    TEAL as teal,
    CYAN as cyan,
    BLUE as blue,
    PURPLE as purple,
    MAGENTA as magenta,
    WHITE as white,
    BLACK as black,
    GOLD as gold,
    PINK as pink,
    AQUA as aqua,
    JADE as jade,
    AMBER as amber,
    OLD_LACE as old_lace
)

class LEDService(Node):

    def __init__(self):
        super().__init__('led_service')
        self.led_pixel_service = self.create_service(LEDPixelColor, '~/set_led_pixel_color', self.set_led_pixel_callback)
        self.led_color_service = self.create_service(LEDRingColor, '~/set_led_ring_color', self.set_led_ring_callback)
        self.pixel_pin = board.D12
        self.num_pixels = 45
        self.pixel_order = neopixel.GRB
        self.pixels = neopixel.NeoPixel(self.pixel_pin, self.num_pixels, brightness=0.2, auto_write=False, pixel_order=self.pixel_order)


    def set_led_pixel_callback(self, request, response):
        self.get_logger().info(str(request))
        self.pixels[request.index] = (request.r, request.g, request.b)
        self.pixels.show()
        response.success = True
        return response

    def set_led_ring_callback(self, request, response):
        self.get_logger().info(str(request))
        try:
            color = eval(request.color)
            solid = Solid(self.pixels, color=color)
            solid.animate()
            response.success = True
            response.message = "Successfully set LED ring color"
        except:
            response.success = False
            response.message = "LED ring color not available"

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