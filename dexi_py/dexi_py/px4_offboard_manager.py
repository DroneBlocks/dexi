import sys
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from dexi_interfaces.msg import OffboardNavCommand
from px4_msgs.msg import VehicleCommand, VehicleStatus, VehicleGlobalPosition, VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint
from threading import Thread
from pysm import State, StateMachine, Event
from queue import Queue
from enum import Enum
from .utils.timer import Timer

class PX4OffboardManager(Node):
    def __init__(self):
        ####################### R O S  N O D E  S E T U P ############################
        super().__init__('px4_offboard_manager') #type: ignore
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
        self.vehicle_global_pos_subscriber = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_pos_callback, qos_profile)
        self.vehicle_local_pos_subscriber = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_pos_callback, qos_profile)

        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.vehicle_offboard_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)

        # Subscriber listening for offboard commands
        self.offboard_manager = self.create_subscription(OffboardNavCommand, "offboard_manager", self.handle_offboard_command, qos_profile)

        exec_frequency = 20 # Hz
        timer_period = 1/exec_frequency  # seconds
        self.frame_timer = self.create_timer(timer_period, self.exec_frame)

        # self.offboard_manager = OffboardManager(self.vehicle_offboard_mode_publisher, self.trajectory_setpoint_publisher,self.enable_offboard_mode)
        ###############################################################################

        #################### P X 4  S T A T E  S T O R A G E ##########################
        self.prev_vehicle_status_msg = None
        self.lat = 0.00
        self.lon = 0.00
        self.alt = 0.00

        self.x = 0.00
        self.y = 0.00
        self.z = 0.00

        self.heading = 0.00

        self.offboard_heartbeat = OffboardControlMode()
        self.offboard_heartbeat.position=True

        self.offboard_heartbeat_thread = Thread(target=self.send_offboard_heartbeat, args=())
        self.offboard_heartbeat_thread.daemon = True
        self.offboard_heartbeat_thread_run_flag = False
        self.offboard_heartbeat_thread.start()

        self.offboard_timer = Timer()
        self.offboard_timer.set_timeout(1)
        self.offboard_timer.function = self.enable_offboard_mode


        ###############################################################################

        #################### S T A T E  M A C H I N E  S E T U P ######################
        # event queue
        self.event_queue = Queue()

        # # message in queues
        # self.vehicle_status_queue = Queue()
        # self.vehicle_global_pos_queue = Queue()

        # # message out queues
        # self.msg_out_queue = Queue()

        # create the init state
        self.init_state = State('init')
        
        # create the Disarm state
        self.disarm_state = State('Disarm')
        self.disarm_state.handlers = {
            'enter': self.disarm_enter,
            'arm_event': self.handle_arm_request_event
        }
        
        # create the Arm state
        self.arm_state = State('Arm')
        self.arm_state.handlers = {
            'enter': self.arm_enter,
            'disarm_event': self.handle_disarm_request_event,
            'takeoff_requested_event':self.handle_takeoff_request_event
        }
        
        # create the Takeoff state
        self.takeoff_state = State('Takeoff')
        
        # create the Land state
        self.landing_state = State('Land')

        # create the Hold state
        self.hold_state = State('Hold')
        self.hold_state.handlers = {
            'enter': self.hold_enter
        }

        # create the navigate state
        self.navigate_state = State('Navigate') 
        self.navigate_state.handlers = {
            'up_event':  lambda state, event: self.fly_up(1), 
            'down_event': lambda state, event: self.fly_down(1),
            'right_event': lambda state, event: self.fly_right(1),
            'left_event': lambda state, event: self.fly_left(1),
            'forward_event': lambda state, event: self.fly_forward(1),
            'backward_event': lambda state, event: self.fly_backward(1),
            'yaw_left_event': lambda state, event: self.yaw_left(90),
            'yaw_right_event': lambda state, event: self.yaw_right(90),
            'land_event': lambda state, event: self.land()
        }

        # create the state machine
        self.sm = StateMachine('sm')
        
        # add the states
        self.sm.add_state(self.init_state, initial=True)
        self.sm.add_state(self.disarm_state)
        self.sm.add_state(self.arm_state)
        self.sm.add_state(self.takeoff_state)
        self.sm.add_state(self.landing_state)
        self.sm.add_state(self.hold_state)
        self.sm.add_state(self.navigate_state)
        
        # add the transitions
        self.sm.add_transition(self.init_state, self.disarm_state, events=['vehicle_status_init_event'])
        self.sm.add_transition(self.disarm_state, self.arm_state, events=['vehicle_armed_event'])
        self.sm.add_transition(self.arm_state, self.takeoff_state, events=['vehicle_takeoff_event'])
        self.sm.add_transition(self.takeoff_state, self.landing_state, events=['vehicle_begin_landing_event'])
        self.sm.add_transition(self.takeoff_state, self.hold_state, events=['vehicle_hold_event'])
        self.sm.add_transition(self.hold_state, self.navigate_state, events=["vehicle_begin_navigation_event"])
        self.sm.add_transition(self.hold_state, self.landing_state, events=['vehicle_begin_landing_event'])
        self.sm.add_transition(self.navigate_state, self.hold_state, events=['vehicle_hold_event'])
        self.sm.add_transition(self.navigate_state, self.landing_state, events=['vehicle_begin_landing_event'])
        self.sm.add_transition(self.landing_state, self.disarm_state, events=['vehicle_disarmed_event'])

        self.sm.initialize()
        ###############################################################################
        
    #################### S T A T E  M A C H I N E  M E T H O D S ######################
    def disarm_enter(self, state, event):
        self.offboard_heartbeat_thread_run_flag = False
        self.enable_hold_mode()

    def arm_enter(self, state, event):
        # self.takeoff(5)
        pass

    def hold_enter(self, state, event):
        self.offboard_heartbeat_thread_run_flag = True
        self.send_trajectory_setpoint_position(0,0,0)
        self.offboard_timer.start()
        self.event_queue.put(Event("vehicle_begin_navigation_event"))
    
    def handle_arm_request_event(self, state, event):
        self.arm()
    
    def handle_disarm_request_event(self, state, event):
        self.disarm()
    
    def handle_takeoff_request_event(self, state: State, event: Event):
        self.get_logger().info('handle_takeoff_request_event line 181')
        self.takeoff(event.cargo["altitude"])
    ####################################################################################

    ########################### R O S  N O D E  M E T H O D S ##########################
    def exec_frame(self):
        old_state = self.sm.state.name # type: ignore
        new_state = old_state
        while not self.event_queue.empty():
            event = self.event_queue.get()
            self.get_logger().info(f"Got a {event.name}")
            self.sm.dispatch(event)
            self.get_logger().info('done dispatching')
            new_state = self.sm.state.name # type: ignore
            if old_state != new_state:
                self.get_logger().info(f"TRANSITIONED FROM: ({old_state}) ---> ({new_state})") # type: ignore

    
    def vehicle_status_callback(self, msg: VehicleStatus):
        
        if self.prev_vehicle_status_msg == None:
            self.event_queue.put(Event("vehicle_status_init_event"))
        else:
            #check arming
            if self.prev_vehicle_status_msg.arming_state != msg.arming_state:
                if msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED: #disarmed
                    self.event_queue.put(Event("vehicle_disarmed_event"))
                elif msg.arming_state == VehicleStatus.ARMING_STATE_ARMED: # armed
                    self.event_queue.put(Event("vehicle_armed_event"))
            #check nav_state
            if self.prev_vehicle_status_msg.nav_state != msg.nav_state:
                if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: # takeoff
                    self.event_queue.put(Event("vehicle_takeoff_event"))
                elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND: # land
                    self.event_queue.put(Event("vehicle_begin_landing_event"))
                elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: # hold
                    self.event_queue.put(Event("vehicle_hold_event"))
                elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: # offboard
                    self.event_queue.put(Event("vehicle_offboard_event"))
        # self.get_logger().info("GOT A MESSAGE")
        self.prev_vehicle_status_msg = msg
    
    def vehicle_global_pos_callback(self, msg: VehicleGlobalPosition):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
        # self.get_logger().info(f"Lat: {self.lat} Lon: {self.lon} Alt: {self.alt}")
    
    def vehicle_local_pos_callback(self, msg: VehicleLocalPosition):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.heading = msg.heading
        # self.get_logger().info(f"Lat: {self.lat} Lon: {self.lon} Alt: {self.alt}")

    # Receive messages published to /offboard_manager
    def handle_offboard_command(self, msg: OffboardNavCommand):
        distance_or_degrees = msg.distance_or_degrees or 1
        
        # Dynamically call the method if it exists
        command_method = getattr(self, msg.command, None)

        if callable(command_method):
            # Pass the distance_or_degrees argument if the method requires it
            if msg.command in [
                'takeoff', 'fly_forward', 'fly_backward', 'fly_left', 
                'fly_right', 'fly_up', 'fly_down', 'yaw_left', 'yaw_right'
            ]:
                command_method(distance_or_degrees)
            else: # Arm, Land and other cases with no distance
                command_method()
        else:
            self.get_logger().warn(f"Command '{msg.command}' is not recognized.")

    ###################################################################################

    ########################### P X 4  R O S  M E T H O D S ###########################
    def get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

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

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 # arm (0 disarm 1 arm)
        msg.param2 = float(0) # force (0 for no 21196 for force)
        self.send_vehicle_command(msg)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0 # arm (0 disarm 1 arm)
        msg.param2 = float(0) # force (0 for no 21196 for force)
        self.send_vehicle_command(msg)
        self.get_logger().info('Disarm command sent')

    def takeoff(self, alt):
        '''
        take off to the specified MSL altitude
        '''
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param1 = float(-1)
        msg.param2 = float(0)
        msg.param3 = float(0)
        msg.param4 = float('nan')
        msg.param5 = float('nan')
        msg.param6 = float('nan')
        msg.param7 = float(alt + self.alt) 

        self.send_vehicle_command(msg)

        self.get_logger().info('Takeoff command sent')
    
    def go_to_location(self, lat, lon, alt_msl, ground_speed = 5):
        '''
        Reposition the vehicle to a specific WGS84 global position.
        '''
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_REPOSITION
        msg.param1 = float(ground_speed) # ground speed
        msg.param2 = 1.0 # bitmask
        msg.param3 = float('nan') # loiter radius for planes
        msg.param4 = float('nan') # yaw heading
        msg.param5 = float(lat) # lat
        msg.param6 = float(lon) # lon
        msg.param7 = float(alt_msl)# alt

        self.vehicle_command_publisher.publish(msg)

        self.get_logger().info('Reposition command sent')
    
    def enable_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = float(1) # magic number? 
        msg.param2 = float(6) # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self.send_vehicle_command(msg)
    
    def enable_hold_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = float(1) # magic number? 
        msg.param2 = float(4) # PX4_CUSTOM_MAIN_MODE_AUTO
        msg.param3 = float(3)
        self.send_vehicle_command(msg)

    def send_offboard_heartbeat(self):
        while True:
            if self.offboard_heartbeat_thread_run_flag == True:
                self.vehicle_offboard_mode_publisher.publish(self.offboard_heartbeat)
            time.sleep(1/20)

    def send_trajectory_setpoint_position(self, x, y , z, yaw = None):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
        msg.position = [float(x), float(y) , float(z)]
        if yaw == None:
            yaw=self.heading
        msg.yaw = yaw
        self.get_logger().info(str(msg))
        self.trajectory_setpoint_publisher.publish(msg)

    def send_trajectory_setpoint_velocity(self, vx, vy , vz):
        msg = TrajectorySetpoint()
        msg.velocity = [float(vx), float(vy) , float(vz)]
        self.trajectory_setpoint_publisher.publish(msg)

    def fly_forward(self, distance):

        new_x = distance * math.cos(self.heading) + self.x
        new_y = distance * math.sin(self.heading) + self.y
        
        self.send_trajectory_setpoint_position(new_x, new_y, self.z)
    
    def fly_backward(self, distance):
        new_x = distance * math.cos(self.heading + math.pi) + self.x
        new_y = distance * math.sin(self.heading + math.pi) + self.y
        
        self.send_trajectory_setpoint_position(new_x, new_y, self.z)
    
    def fly_right(self, distance):
        new_x = distance * math.cos(self.heading + (math.pi/2)) + self.x
        new_y = distance * math.sin(self.heading + (math.pi/2)) + self.y
        
        self.send_trajectory_setpoint_position(new_x, new_y, self.z)
    
    def fly_left(self, distance):
        new_x = distance * math.cos(self.heading + (-1 * math.pi/2)) + self.x
        new_y = distance * math.sin(self.heading + (-1 * math.pi/2)) + self.y

        self.send_trajectory_setpoint_position(new_x, new_y, self.z)
    
    def fly_up(self, distance):
        z_pos = self.z + -1*distance
        self.send_trajectory_setpoint_position(self.x, self.y, z_pos)
    
    def fly_down(self, distance):
        z_pos = self.z + distance
        self.send_trajectory_setpoint_position(self.x, self.y, z_pos)

    def yaw_left(self, angle):
        hdg = self.heading - math.radians(angle)
        self.send_trajectory_setpoint_position(self.x, self.y, self.z, hdg)

    def yaw_right(self, angle):
        hdg = self.heading + math.radians(angle)
        self.send_trajectory_setpoint_position(self.x, self.y, self.z, hdg)

    def land(self):
        '''
        switch to land mode
        '''
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.send_vehicle_command(msg)
        self.get_logger().info('Land command sent')
    ###################################################################################
    

def main(args=None):
    rclpy.init(args=args)
    try:
        px4_offboard_manager = PX4OffboardManager()
        rclpy.spin(px4_offboard_manager)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        px4_offboard_manager.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

if __name__ == '__main__':
    main()
