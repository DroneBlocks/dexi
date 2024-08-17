import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import RPi.GPIO as GPIO
from dexi_msgs.srv import GPIOSend

class GPIOWriterService(Node):

    def __init__(self):
        super().__init__('gpio_writer_service')
        self.gpio_send_service = self.create_service(GPIOSend, '~/send_gpio_pin', self.send_gpio_pin_callback)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def send_gpio_pin_callback(self, request, response):
        self.get_logger().info(str(request))
        try:
            GPIO.setup(request.pin, GPIO.OUT)
            GPIO.output(request.pin, request.state)
            response.success = True
            response.message = "Successfully set pin {} as {}".format(request.pin, request.state)
        except:
            response.success = False
            response.message = "Error setting pin {} as {}".format(request.pin, request.state)

        return response

def main(args=None):
    rclpy.init(args=args)
    try:
        gpio_writer_service = GPIOWriterService()
        rclpy.spin(gpio_writer_service)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        GPIO.cleanup()
        gpio_writer_service.destroy_node()
        sys.exit(1)

if __name__ == '__main__':
    main()