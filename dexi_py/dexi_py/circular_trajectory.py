import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
import math
import time

class CircularTrajectoryNode(Node):
    def __init__(self):
        super().__init__('circular_trajectory_node')
        self.publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory_setpoint)  # 10 Hz
        self.start_time = time.time()
        
        # Circle parameters
        self.radius = 3.0  # meters
        self.center_x = 0.0
        self.center_y = 0.0
        self.altitude = -5.0  # meters
        self.angular_velocity = 0.1  # radians per second (speed of the orbit)
    
    def publish_trajectory_setpoint(self):
        current_time = time.time() - self.start_time
        angle = self.angular_velocity * current_time

        # Calculate position
        x = self.center_x + self.radius * math.cos(angle)
        y = self.center_y + self.radius * math.sin(angle)
        z = self.altitude

        # Calculate yaw
        yaw = math.atan2(self.center_y - y, self.center_x - x)

        # Create and publish TrajectorySetpoint message
        setpoint = TrajectorySetpoint()
        setpoint.position = [x, y, z]
        setpoint.velocity = [
            -self.radius * self.angular_velocity * math.sin(angle),  # vx
            self.radius * self.angular_velocity * math.cos(angle),  # vy
            0.0  # vz
        ]
        setpoint.acceleration = [0.0, 0.0, 0.0]  # Optional
        setpoint.yaw = yaw
        setpoint.yawspeed = 0.0  # No yaw rate change needed

        self.publisher.publish(setpoint)
        self.get_logger().info(f'Published Trajectory Setpoint: x={x}, y={y}, z={z}, yaw={yaw}')

def main(args=None):
    rclpy.init(args=args)
    node = CircularTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
