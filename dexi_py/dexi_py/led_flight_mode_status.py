import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import IntEnum

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus

import board
import neopixel

from adafruit_led_animation.animation.solid import Solid
from adafruit_led_animation.color import (
    PURPLE,
    WHITE,
    GOLD,
    JADE,
    TEAL,
    PINK,
    MAGENTA,
    ORANGE,
    RED,
    YELLOW,
    GREEN,
    CYAN,
    AQUA,
    BLUE,
    BLACK
)

# These map to PX4 msgs VechicleStatus nav_state
class FlightMode(IntEnum):
    MANUAL = 0
    ALTITUDE = 1
    POSITION = 2
    MISSION = 3
    HOLD = 4
    RTL = 5
    SLOW = 6
    FREE5 = 7
    FREE4 = 8
    FREE3 = 9
    ACRO = 10
    FREE2 = 11
    DESCEND = 12
    TERMINATION = 13
    OFFBOARD = 14
    STABILIZED = 15
    FREE1 = 16
    TAKEOFF = 17
    LAND = 18
    TARGET = 19
    PRECLAND = 20
    ORBIT = 21
    VTOL_TAKOFF = 22

class LEDFlightModeStatus(Node):

    def __init__(self):
        super().__init__('led_flight_mode')
        self.pixel_pin = board.D12
        self.num_pixels = 45
        self.pixel_order = neopixel.GRB
        self.previous_flight_mode = FlightMode.MANUAL
        self.current_flight_mode = FlightMode.MANUAL
        self.pixels = neopixel.NeoPixel(self.pixel_pin, self.num_pixels, brightness=0.2, auto_write=False, pixel_order=self.pixel_order)
        
        solid = Solid(self.pixels, color=BLACK)
        solid.animate()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.listener_callback,
            qos_profile)

    def listener_callback(self, msg):

        self.current_flight_mode = FlightMode(msg.nav_state)

        if(self.current_flight_mode != self.previous_flight_mode):
            if self.current_flight_mode == FlightMode.STABILIZED:
                solid = Solid(self.pixels, color=WHITE)
                solid.animate()
            elif self.current_flight_mode == FlightMode.ALTITUDE:
                solid = Solid(self.pixels, color=YELLOW)
                solid.animate()
            elif self.current_flight_mode == FlightMode.POSITION:
                solid = Solid(self.pixels, color=GREEN)
                solid.animate()
            else:
                self.get_logger().info('FLIGHT MODE COLOR NOT YET SUPPORTED')
                solid = Solid(self.pixels, color=BLACK)
                solid.animate()

            self.previous_flight_mode = self.current_flight_mode


def main(args=None):
    rclpy.init(args=args)
    led = LEDFlightModeStatus()
    rclpy.spin(led)
    led.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
