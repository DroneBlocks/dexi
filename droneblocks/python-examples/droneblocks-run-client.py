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
from time import sleep
print('one')
sleep(3)
print('two')
sleep(3)
print('three')
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


