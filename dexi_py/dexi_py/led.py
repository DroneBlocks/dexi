import rclpy
from rclpy.node import Node
from led_msgs.srv import SetLED
import board
import neopixel

class LEDService(Node):

    def __init__(self):
        super().__init__('led_service')
        self.srv = self.create_service(SetLED, 'set_led', self.set_led_callback)
        self.pixel_pin = board.D10
        self.num_pixels = 30
        self.pixel_order = neopixel.GRB
        self.pixels = neopixel.NeoPixel(self.pixel_pin, self.num_pixels, brightness=0.2, auto_write=False, pixel_order=self.pixel_order)


    def set_led_callback(self, request, response):
        self.get_logger().info(str(request))
        self.pixels[request.index] = (request.r, request.g, request.b)
        self.pixels.show()
        response.success = True
        return response

    def set_leds_callback(self):
        return

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