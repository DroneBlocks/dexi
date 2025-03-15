import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode

class OffboardHeartbeatNode(Node):
    def __init__(self):
        super().__init__('offboard_heartbeat_node')
        
        # Create a publisher for the OffboardControlMode topic
        self.offboard_control_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        
        # Timer to publish at a regular interval
        self.timer = self.create_timer(0.1, self.publish_offboard_control_mode)

    def publish_offboard_control_mode(self):
        # Create and populate the OffboardControlMode message
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert nanoseconds to microseconds
        msg.position = True

        # Publish the message
        self.offboard_control_publisher.publish(msg)
        self.get_logger().info('Published OffboardControlMode message')

def main(args=None):
    rclpy.init(args=args)
    node = OffboardHeartbeatNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
