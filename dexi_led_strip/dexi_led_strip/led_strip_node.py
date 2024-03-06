import time
from typing import Any
from threading import Thread, Event

import rclpy
from rclpy.node import Node

from adafruit_led_animation.animation.solid import Solid
from adafruit_led_animation.animation.rainbow import Rainbow
from adafruit_led_animation.animation.rainbowchase import RainbowChase
from adafruit_led_animation.animation.pulse import Pulse
from adafruit_led_animation.animation.comet import Comet

from dexi_msgs.srv import SetLedEffect

from dexi_led_strip.util import SPI, run_anim_until_done
from dexi_led_strip.neopixel_ring_spi import NeoPixelRing_SPI
from dexi_led_strip.channel_wrap_animation import ChannelWrapAnim


class LEDStripNode(Node):
    def __init__(self) -> None:
        super().__init__('dexi_led_strip')

        self.declare_parameter('spi_port', 1)
        spi = SPI[self.get_parameter('spi_port').value]()

        self.declare_parameter('led_count', 20)  # Actually 22
        led_count = self.get_parameter('led_count').value

        self.declare_parameter('start_index', 0)  # Actually 16
        start_index = self.get_parameter('start_index').value

        self.declare_parameter('pixel_order', 'GRB')
        pixel_order = self.get_parameter('pixel_order').value

        channel_count = len(pixel_order)

        self.set_service = self.create_service(SetLedEffect, 'set', self.set_callback)

        self.pixels = NeoPixelRing_SPI(
            spi,
            led_count,
            pixel_order=pixel_order,
            brightness=1,
            auto_write=False,
            start_index=start_index
        )

        self.boot_anim = ChannelWrapAnim(self.pixels, 0.05)

        self.base_color = [0] * channel_count

        self._current_animation = None
        self.animation_duration = float('nan')
        self._animation_start_time = -1
        self.animation_changed = Event()

        self.update_thread = Thread(target=self.update_loop, daemon=True)
        self.play_boot_anim()
        self.update_thread.start()

    @property
    def current_animation(self) -> Any:
        return self._current_animation

    @current_animation.setter
    def current_animation(self, new_animation: Any) -> None:
        self._current_animation = new_animation
        self._animation_start_time = time.time()
        self.animation_changed.set()

    def play_boot_anim(self) -> None:
        if not self.update_thread.is_alive():
            self.pixels.fill(0)
            run_anim_until_done(self.boot_anim)

    def set_callback(self, request: SetLedEffect.Request, response: SetLedEffect.Response) -> SetLedEffect.Response:
        # ToDo: add speed and argument to msg
        color = request.r, request.g, request.b
        if request.effect == 'base':
            self.current_animation = None
        elif request.effect in ('', 'solid'):
            self.current_animation = Solid(self.pixels, color)
        elif request.effect == 'rainbow':
            self.animation_duration = request.duration
            self.current_animation = Rainbow(self.pixels, 0.05)##
        elif request.effect =='rainbowchase':
            self.animation_duration = request.duration
            self.current_animation = RainbowChase(self.pixels, 0.05)
        elif request.effect =='pulse':
            self.animation_duration = request.duration
            self.current_animation = Pulse(self.pixels, 0.05, color)
        elif request.effect =='comet':
            self.animation_duration = request.duration
            self.current_animation = Comet(self.pixels, 0.05, color)
        else:
            response.success = False
            response.message = f'Unknown effect: \"{request.effect}\"'
            return response
        response.success = True
        response.message = 'Success'
        return response

    def update_loop(self) -> None:
        animation = self.current_animation
        while True:
            if self.animation_changed.is_set():
                self.animation_changed.clear()
                animation = self.current_animation

            if animation is not None:
                animation.animate()
            else:
                self.pixels.fill(self.base_color)
            
            if isinstance(animation, Solid) or animation is None:
                self.animation_changed.wait(self.animation_duration)
            elif (time.time() - self._animation_start_time) >= self.animation_duration:
                self.current_animation = None


def main(args=None):
    rclpy.init(args=args)
    led_strip_node = LEDStripNode()
    rclpy.spin(led_strip_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
