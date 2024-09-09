import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import RPi.GPIO as GPIO
from dexi_interfaces.srv import GPIOSend

DEXI_GPIO_PINS = (
    16,
    17,
    20,
    21,
    22,
    23,
    24
)

class GPIOWriterService(Node):

    def __init__(self):
        super().__init__('gpio_writer_service')
        self.declare_parameter('gpio_outputs', rclpy.Parameter.Type.INTEGER_ARRAY)
        gpio_outputs = self.get_parameter('gpio_outputs').value

        # We have gpio outputs configured and loaded from config/gpio.yaml
        if len(gpio_outputs) > 0:

            # Configure with BCM pin numbering
            GPIO.setmode(GPIO.BCM)

            # Disable warnings
            GPIO.setwarnings(False)

            # Loop and setup the GPIO
            for gpio_pin in gpio_outputs:

                # Set pin as output
                GPIO.setup(gpio_pin, GPIO.OUT)

                self.get_logger().info(str(gpio_pin))
                self.gpio_send_service = self.create_service(GPIOSend, '~/write_gpio_{}'.format(str(gpio_pin)), self.send_gpio_pin_callback)
        

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
        rclpy.shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()