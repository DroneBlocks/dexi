import rclpy
from rclpy.node import Node
from time import sleep
from dexi_msgs.srv import Run


class DBRunClient(Node):
  def __init__(self):
    super().__init__('droneblocks_run_client')
    self.run_client = self.create_client(Run, '/droneblocks/run')

    while not self.run_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

    self.request = Run.Request()

  def run_mission(self):
    self.request.code = """
import rclpy
from led_msgs.srv import SetLED
from time import sleep

node = rclpy.create_node('temp')

set_led = node.create_client(SetLED, '/dexi/set_led_mock')

while not set_led.wait_for_service(timeout_sec=1.0):
  node.get_logger().info('service not available, waiting again...')

for index in range(0, 20):
  request = SetLED.Request()
  request.index = index
  request.r = 0
  request.g = 255
  request.b = 0
  request.brightness = 255
  sleep(0.1)

  print(str(index))
  future = set_led.call_async(request)
  rclpy.spin_until_future_complete(node, future)
  print(str(future.result()))

print('done')
    """
    future = self.run_client.call_async(self.request)
    rclpy.spin_until_future_complete(self, future)
    return future.result()

def main(args=None):
  rclpy.init(args=args)

  db_client = DBRunClient()

  response = db_client.run_mission()

  db_client.get_logger().info(str(response))

  # db_client.get_logger().info(
  #     'Result of add_two_ints: for %d + %d = %d' %
  #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))

  db_client.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()


