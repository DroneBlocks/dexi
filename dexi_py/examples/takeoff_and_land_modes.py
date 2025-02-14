import rclpy
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition, VehicleCommand
from threading import Thread

class TakeoffAndLand(Node):
    def __init__(self):
        super().__init__('takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Where we send flight modes such as arm, takeoff, land
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        
        # Get global position coordinates
        self.vehicle_global_pos_subscriber = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_pos_callback, qos_profile)

        # Local position coordinates
        self.altitude = 0.00

        # Sleep 5 seconds before arming
        time.sleep(5.0)

        # Arm the drone
        self.arm()

        # Sleep for 1 second
        time.sleep(1.0)

        # Takeoff to 3 meters AGL
        self.takeoff(3.0)

        # Sleep for 10 seconds
        time.sleep(10.0)

        # Land
        self.land()

    # Send arm command to get motors spinning
    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 # arm (0 disarm 1 arm)
        msg.param2 = float(0)
        self.send_vehicle_command(msg)
        self.get_logger().info('Arm command sent')

    # Send takeoff mode command
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

    # Send land mode command
    def land(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.send_vehicle_command(msg)
        self.get_logger().info('Land command sent')

    def send_vehicle_command(self, msg: VehicleCommand):
        msg.timestamp = self.get_timestamp()
        
        # if the msg has a bunch of defaults set, set them to our defaults
        if all([
                msg.target_system == 0,
                msg.target_component == 0,
                msg.source_system == 0,
                msg.source_component == 0
                ]):
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            
        msg.from_external = True
        
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(f'Sent command {msg.command}')

    def get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    # Used to grab altitude for takeoff command since it will require a MSL altitude
    def vehicle_global_pos_callback(self, msg: VehicleGlobalPosition):
        self.alt = msg.alt

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffAndLand()
    
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
