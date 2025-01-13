import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
import math
import time


class YawInPlaceNode(Node):
    def __init__(self):
        super().__init__('yaw_in_place_node')
        self.publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory_setpoint)  # 10 Hz
        self.start_time = time.time()

        # Yaw rate in radians per second
        self.yaw_rate = math.radians(30)  # 30 degrees per second

    def publish_trajectory_setpoint(self):
        current_time = time.time() - self.start_time

        # Create and publish TrajectorySetpoint message
        setpoint = TrajectorySetpoint()
        setpoint.position = [float('nan'), float('nan'), -3.0]  # Hold position
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]  # No velocity control
        setpoint.acceleration = [0.0, 0.0, 0.0]  # Optional
        setpoint.yaw = float('nan')  # Hold current yaw
        setpoint.yawspeed = self.yaw_rate * -1 # Counter-clockwise yaw rate

        self.publisher.publish(setpoint)
        self.get_logger().info(f'Published yaw rate: {math.degrees(self.yaw_rate)} degrees/sec')

def main(args=None):
    rclpy.init(args=args)
    node = YawInPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
