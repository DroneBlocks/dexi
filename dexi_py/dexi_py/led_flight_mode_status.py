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

class FlightMode(IntEnum):
    DEFAULT = 0
    STABILIZED = 15
    ALTITUDE = 1
    POSITION = 2
    HOLD = 4

class LEDFlightModeStatus(Node):

    def __init__(self):
        super().__init__('led_flight_mode')
        self.pixel_pin = board.D12
        self.num_pixels = 45
        self.pixel_order = neopixel.GRB
        self.previous_flight_mode = FlightMode.DEFAULT
        self.current_flight_mode = FlightMode.DEFAULT
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

        if(msg.nav_state not in iter(FlightMode)):
            self.get_logger().info('FLIGHT MODE NOT FOUND')
            return

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

            self.previous_flight_mode = self.current_flight_mode


def main(args=None):
    rclpy.init(args=args)
    led = LEDFlightModeStatus()
    rclpy.spin(led)
    led.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
