import rclpy
from led_msgs.srv import SetLED
from time import sleep
# from led_msgs.srv import SetLEDs
# from led_msgs.msg import LEDState

rclpy.init()

node = rclpy.create_node('temp')

# set_leds = node.create_client(SetLEDs, 'set_leds')
set_led = node.create_client(SetLED, '/dexi/set_led')

# while not set_leds.wait_for_service(timeout_sec=1.0):
 # node.get_logger().info('service not available, waiting again...')

while not set_led.wait_for_service(timeout_sec=1.0):
  node.get_logger().info('service not available, waiting again...')

# state = LEDState()
# state.index = 0
# state.r = 255
# state.g = 0
# state.b = 0
# state.brightness = 255

# request = SetLEDs.Request()
# request.leds = [state]

# future = set_leds.call_async(request)

for index in range(0, 30):    
  request = SetLED.Request()
  request.index = index
  request.r = 0
  request.g = 0
  request.b = 0
  request.brightness = 255
  sleep(0.1)

  future = set_led.call_async(request)

  rclpy.spin_until_future_complete(node, future)

  print(future.result())

print('done')

