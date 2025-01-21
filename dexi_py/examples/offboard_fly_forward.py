import rclpy
import time
import math
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleGlobalPosition, VehicleCommand
from threading import Thread

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

        # Publisher to switch flight modes between takeoff, offboard, and land
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        # Send setpoints for navigation such as fly_forward
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # Get local position coordinates
        self.vehicle_local_pos_subscriber = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_pos_callback, qos_profile)

        # Get global position coordinates
        self.vehicle_global_pos_subscriber = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_pos_callback, qos_profile)

        # Subscriber listening for offboard navigation commands
        self.offboard_manager = self.create_subscription(String, "offboard_manager", self.launch_mission, qos_profile)

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

    # Offboard heartbeat thread running at 10Hz
    def send_offboard_heartbeat(self):
        while True:
            if self.offboard_heartbeat_thread_run_flag == True:
                self.publish_offboard_control_mode()
            time.sleep(1/10)

    # Send offboard control mode message
    def publish_offboard_control_mode(self):
        # Create and populate the OffboardControlMode message
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        msg.position = True

        # Publish the message
        self.offboard_control_publisher.publish(msg)

    # Send arm command to get motors spinning
    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 # arm (0 disarm 1 arm)
        msg.param2 = float(0)
        self.send_vehicle_command(msg)
        self.get_logger().info('Arm command sent')

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

    # Switch to offboard mode
    def enable_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = float(1)
        msg.param2 = float(6) # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self.send_vehicle_command(msg)

     # Send land mode command
    def land(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.send_vehicle_command(msg)
        self.get_logger().info('Land command sent')

    # Toggle flight modes such as takeoff, offboard, and land
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
    
    # Send location position coordinates for offboard navigation
    def send_trajectory_setpoint_position(self, x, y , z, yaw = None):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
        msg.position = [float(x), float(y) , float(z)]
        if yaw == None:
            yaw = self.heading
        msg.yaw = yaw
        self.get_logger().info(str(msg))
        self.trajectory_setpoint_publisher.publish(msg)

    # Fly forward a user specified distance in meters
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

    # Timestamp for ROS messages
    def get_timestamp(self):
        return self.get_clock().now().nanoseconds // 1000
    
    # Callback method that launches the mission when a "launch" message is received on the offboard_manager topic
    def launch_mission(self, msg: String):
        if msg.data == "launch":
            threading.Thread(target=lambda: (
            self.arm(),
            time.sleep(1),
            self.takeoff(5),
            time.sleep(10),
            self.enable_offboard_mode(),
            time.sleep(1),
            self.fly_forward(20),
            time.sleep(10),
            self.land()
        )).start()
        else:
            self.get_logger().info("Unknown command type")

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
