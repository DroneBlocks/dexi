import rclpy
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleGlobalPosition, VehicleCommand
from threading import Thread
from dexi_interfaces.msg import OffboardNavCommand

class OffboardFlyForward(Node):
    def __init__(self):
        super().__init__('fly_forward')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create a publisher for the OffboardControlMode topic
        self.offboard_control_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        # Send setpoints for takeoff and land
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # Get local position coordinates
        self.vehicle_local_pos_subscriber = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_pos_callback, qos_profile)

        # Get global position coordinates
        self.vehicle_global_pos_subscriber = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_pos_callback, qos_profile)

        # Create a thread to publish the heartbeat signal
        self.offboard_heartbeat_thread = Thread(target=self.send_offboard_heartbeat, args=())
        self.offboard_heartbeat_thread.daemon = True
        self.offboard_heartbeat_thread_run_flag = True
        self.offboard_heartbeat_thread.start()

        # Local position coordinates
        self.x = 0.00
        self.y = 0.00
        self.z = 0.00
        self.heading = 0.00
        self.altitude = 0.00

        time.sleep(10)

        self.takeoff(5.0)

        # self.get_logger().info('after flying forward')

    def send_offboard_heartbeat(self):
        while True:
            if self.offboard_heartbeat_thread_run_flag == True:
                self.publish_offboard_control_mode()
            time.sleep(1/10)

    def publish_offboard_control_mode(self):
        # Create and populate the OffboardControlMode message
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert nanoseconds to microseconds
        msg.position = True

        # Publish the message
        self.offboard_control_publisher.publish(msg)
        #self.get_logger().info('Published OffboardControlMode message')

    # Send the explicit takeoff command mode
    def takeoff(self, altitude):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param1 = float(-1)
        msg.param2 = float(0)
        msg.param3 = float(0)
        msg.param4 = float('nan')
        msg.param5 = float('nan')
        msg.param6 = float('nan')
        msg.param7 = float(altitude + self.altitude) 

        self.send_vehicle_command(msg)

        self.get_logger().info('Takeoff command sent')


    def send_trajectory_setpoint_position(self, x, y , z, yaw = None):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
        msg.position = [float(x), float(y) , float(z)]
        if yaw == None:
            yaw = self.heading
        msg.yaw = yaw
        self.get_logger().info(str(msg))
        self.trajectory_setpoint_publisher.publish(msg)

    def fly_forward(self, distance):
        new_x = distance * math.cos(self.heading) + self.x
        new_y = distance * math.sin(self.heading) + self.y
        self.send_trajectory_setpoint_position(new_x, new_y, self.z)

    # Used to grab altitude for takeoff command since it will require a MSL altitude
    def vehicle_global_pos_callback(self, msg: VehicleGlobalPosition):
        self.alt = msg.alt

    # Populate local position coordinates
    def vehicle_local_pos_callback(self, msg: VehicleLocalPosition):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.heading = msg.heading

    def get_timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

def main(args=None):
    rclpy.init(args=args)
    node = OffboardFlyForward()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.offboard_heartbeat_thread_run_flag = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
