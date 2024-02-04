import rclpy
from threading import Thread
from rclpy.node import Node
from dexi_msgs.srv import Load

class Mission(Node):

  def __init__(self):
    super().__init__('mission')
    self.cli = self.create_client(Load, '/droneblocks/load')
    while not self.cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting...')
    self.request = Load.Request()

  def execute(self):
    self.future = self.cli.call_async(self.request)

def main(args=None):
  rclpy.init()

  mission = Mission()
  mission.execute()

  while rclpy.ok():
    rclpy.spin_once(mission)
    if mission.future.done():
      try:
        response = mission.future.result()
      except Exception as e:
        mission.get_logger().info(e)
      else:
        mission.get_logger().info(response)
      
      break

  mission.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()