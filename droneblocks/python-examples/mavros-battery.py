import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

SENSOR_QOS = rclpy.qos.qos_profile_sensor_data

class BatteryNode(Node):
    
  def __init__(self):
      super().__init__('battery_state_node')
      self.current_battery = BatteryState()
      self.state_sub = self.create_subscription(BatteryState, '/mavros/battery', self.state_cb, SENSOR_QOS)

  def state_cb(self, msg):
      self.get_logger().info(str(msg))
      self.current_battery = msg

def main(args=None):
    rclpy.init(args=args)
    battery = BatteryNode()
    rclpy.spin(battery)
    battery.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()