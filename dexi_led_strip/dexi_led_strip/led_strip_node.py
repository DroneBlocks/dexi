import time
import math
from typing import Any, NamedTuple
from threading import Thread, Event

import rclpy
from rclpy.node import Node

from adafruit_led_animation.animation import Animation
from adafruit_led_animation.animation.solid import Solid
from adafruit_led_animation.animation.colorcycle import ColorCycle
from adafruit_led_animation.animation.blink import Blink
from adafruit_led_animation.animation.pulse import Pulse
from adafruit_led_animation.animation.chase import Chase
from adafruit_led_animation.animation.comet import Comet
from adafruit_led_animation.animation.rainbow import Rainbow
from adafruit_led_animation.animation.rainbowchase import RainbowChase
from adafruit_led_animation.animation.rainbowcomet import RainbowComet

from dexi_msgs.srv import SetLedEffect

from dexi_led_strip.util import SPI, run_anim_until_done
from dexi_led_strip.neopixel_ring_spi import NeoPixelRing_SPI
from dexi_led_strip.channel_wrap_animation import ChannelWrapAnim


def ensure_non_zero(value: float | int, default: Any = None) -> float | int | Any:
    if math.isclose(value, 0):
        return default
    return value


class AnimationEntry(NamedTuple):
    animation: Animation
    duration: float | int


BASE_COLOR_ENTRY = AnimationEntry(None, None)


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

        self.base_color = [0] * channel_count

        self._current_animation = None
        self._animation_start_time = -1
        self.animation_changed = Event()

        self.update_thread = Thread(target=self.update_loop, daemon=True)
        self.play_boot_anim()
        self.update_thread.start()

    @property
    def current_animation(self) -> AnimationEntry | None:
        return self._current_animation

    @current_animation.setter
    def current_animation(self, new_animation: AnimationEntry) -> None:
        self._current_animation = new_animation
        self._animation_start_time = time.time()
        self.animation_changed.set()

    def play_boot_anim(self) -> None:
        if not self.update_thread.is_alive():
            self.pixels.fill(0)
            boot_anim = ChannelWrapAnim(self.pixels, 0.05)
            run_anim_until_done(boot_anim)

    def set_callback(self, request: SetLedEffect.Request, response: SetLedEffect.Response) -> SetLedEffect.Response:
        # ToDo: add speed and argument to msg
        duration = ensure_non_zero(request.duration)
        effect = request.effect
        color = request.r, request.g, request.b
        if effect == 'base':
            self.current_animation = BASE_COLOR_ENTRY
        elif effect in ('', 'solid'):
            self.current_animation = AnimationEntry(Solid(self.pixels, color), duration)
        elif effect =='cycle':
            self.current_animation = AnimationEntry(ColorCycle(self.pixels, 0.05), duration)
        elif effect =='blink':
            self.current_animation = AnimationEntry(Blink(self.pixels, 0.05, color), duration)
        elif effect =='pulse':
            self.current_animation = AnimationEntry(Pulse(self.pixels, 0.05, color), duration)
        elif effect =='chase':
            self.current_animation = AnimationEntry(Chase(self.pixels, 0.05, color), duration)
        elif effect =='comet':
            self.current_animation = AnimationEntry(Comet(self.pixels, 0.05, color), duration)
        elif effect == 'rainbow':
            self.current_animation = AnimationEntry(Rainbow(self.pixels, 0.05), duration)
        elif effect =='rainbow_chase':
            self.current_animation = AnimationEntry(RainbowChase(self.pixels, 0.05), duration)
        elif effect =='rainbow_comet':
            self.current_animation = AnimationEntry(RainbowComet(self.pixels, 0.05, 3), duration)
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
                start_time = time.time()
                animation, duration = self.current_animation

            if animation is not None:
                animation.animate()
            else:
                self.pixels.fill(self.base_color)
                self.pixels.show()
            
            if isinstance(animation, Solid) or animation is None:
                self.animation_changed.wait(duration)
            elif duration is not None and (time.time() - start_time) >= duration:
                self.current_animation = BASE_COLOR_ENTRY


def main(args=None):
    rclpy.init(args=args)
    led_strip_node = LEDStripNode()
    rclpy.spin(led_strip_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
