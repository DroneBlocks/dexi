import rclpy
from rclpy.node import Node
from led_msgs.srv import SetLED

class LEDMockService(Node):

    def __init__(self):
        super().__init__('led_mock_service')
        self.srv = self.create_service(SetLED, 'set_led_mock', self.set_led_callback)


    def set_led_callback(self, request, response):
        self.get_logger().info(str(request))
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    led_mock_service = LEDMockService()
    rclpy.spin(led_mock_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()