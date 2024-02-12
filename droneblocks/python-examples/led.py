import rclpy
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDState

rclpy.init()

node = rclpy.create_node('temp')

set_leds = node.create_client(SetLEDs, 'set_leds')

while not set_leds.wait_for_service(timeout_sec=1.0):
  node.get_logger().info('service not available, waiting again...')

state = LEDState()
state.index = 0
state.r = 255
state.g = 0
state.b = 0
state.brightness = 255

request = SetLEDs.Request()
request.leds = [state]

future = set_leds.call_async(request)
rclpy.spin_until_future_complete(node, future)

print(future.result())

