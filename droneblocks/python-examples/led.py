import rclpy
from led_msgs.srv import SetLED
from time import sleep

rclpy.init()
node = rclpy.create_node('temp')

set_led = node.create_client(SetLED, '/dexi/set_led_mock')

while not set_led.wait_for_service(timeout_sec=1.0):
  node.get_logger().info('service not available, waiting again...')

for index in range(0, 30):    
  request = SetLED.Request()
  request.index = index
  request.r = 0
  request.g = 255
  request.b = 0
  request.brightness = 255
  sleep(0.1)

  future = set_led.call_async(request)
  rclpy.spin_until_future_complete(node, future)
  print(future.result())

print('done')
