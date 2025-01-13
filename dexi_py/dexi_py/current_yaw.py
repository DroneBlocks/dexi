import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry
from geometry_msgs.msg import PoseStamped
from math import atan2, pi

class CurrentYawNode(Node):
    def __init__(self):
        super().__init__('current_yaw')
        # self.pose_subscriber = self.create_subscription(
        #     PoseStamped,
        #     '/tag_detections',
        #     self.pose_callback,
        #     10
        # )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_subscriber = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )
        # self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.current_yaw = 0.0  # Initialize current yaw from odometry

    def odom_callback(self, msg):
        # Extract current yaw from VehicleOdometry
        self.current_yaw = atan2(2.0 * (msg.q[3] * msg.q[2] + msg.q[0] * msg.q[1]),
                                 1.0 - 2.0 * (msg.q[1] ** 2 + msg.q[2] ** 2))
        
        self.get_logger().info(str(self.current_yaw))
        

    # def pose_callback(self, msg):
    #     # Compute lateral offset error from AprilTag
    #     error_y = msg.pose.position.y
    #     Kp = 0.5  # Proportional gain for yaw adjustment
    #     yaw_adjustment = Kp * error_y

    #     # Compute new desired yaw
    #     desired_yaw = self.current_yaw + yaw_adjustment
    #     desired_yaw = (desired_yaw + pi) % (2 * pi) - pi  # Normalize yaw to [-pi, pi]

    #     # Publish TrajectorySetpoint
    #     trajectory_msg = TrajectorySetpoint()
    #     trajectory_msg.position = [0.0, 0.0, 0.0]  # Maintain current position
    #     trajectory_msg.velocity = [0.0, 0.0, 0.0]  # Zero velocity
    #     trajectory_msg.yaw = desired_yaw  # Desired yaw
    #     trajectory_msg.yawspeed = 0.0  # Optional, can set for smoother control

    #     self.trajectory_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CurrentYawNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
