from threading import Thread, Event

import rclpy
from rclpy.node import Node

from dexi_msgs.srv import SetLedEffect

from util import SPI, run_anim_until_done
from neopixel_ring_spi import NeoPixelRing_SPI
from channel_wrap_animation import ChannelWrapAnim


class LEDStripNode(Node):
    def __init__(self) -> None:
        super().__init__('dexi_led_strip')

        self.declare_parameter('spi_port', 1)
        spi = SPI[self.get_parameter('spi_port').value]

        self.declare_parameter('led_count', 20)  # Actually 22
        led_count = self.get_parameter('led_count').value

        self.declare_parameter('start_index', 0)  # Actually 16
        start_index = self.get_parameter('start_index').value

        self.declare_parameter('pixel_order', 'GRB')
        pixel_order = self.get_parameter('pixel_order').value

        channel_count = len(self.pixel_order)

        self.pixels = NeoPixelRing_SPI(
            spi,
            led_count,
            pixel_order=pixel_order,
            brightness=1,
            auto_write=False,
            start_index=start_index
        )

        self.boot_anim = ChannelWrapAnim(self.pixels, 0.05)

        self.current_animation = None
        self.animation_changed = Event()

        self.update_thread = Thread(target=self.update_loop, daemon=True)
        self.play_boot_anim()
        self.update_thread.start()

    def play_boot_anim() -> None:
        if not self.update_thread.is_alive():
            self.pixels.fill(0)
            run_anim_until_done(self.boot_anim)

    def update_loop() -> None:
        animation = None
        self.animation_changed.set()
        while True:
            if self.animation_changed.is_set():
                self.animation_changed.clear()
                animation = self.current_animation

            if animation is None:
                self.animation_changed.wait()
            else:
                animation.animate()
        

def main(args=None):
    rclpy.init(args=args)
    led_strip_node = LEDStripNode()
    rclpy.spin(led_strip_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
