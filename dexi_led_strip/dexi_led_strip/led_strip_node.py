import sys
import math
import time
from queue import PriorityQueue, Empty
from threading import Thread, Event
from typing import Any, Callable, NamedTuple

import rclpy
from rclpy.node import Node

from dexi_msgs.srv import SetLedEffect

from dexi_led_strip.util import SPI, run_anim_until_done
from dexi_led_strip.neopixel_ring_spi import NeoPixelRing_SPI
from dexi_led_strip.channel_wrap_animation import ChannelWrapAnim

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

RGBColor = tuple[int, int, int]

BOOT_ANIM_SPEED = 0.05
DEFAULT_ANIM_SPEED = 0.05
DEFAULT_ANIM_SIZE = 3
CHASE_SPACING = 3


def ensure_non_zero(value: float | int, default: Any = None) -> float | int | Any:
    if math.isclose(value, 0):
        return default
    return value


class AnimationInfo(NamedTuple):
    """
    Time in seconds between updates (lower is faster)
    """
    speed: float
    """
    RGB color (0-255)
    """
    color: RGBColor
    """
    Reverse the direction of the animation
    """
    reverse: bool
    """
    Length or size in pixels of a trail or line
    """
    size: int


class AnimationEntry(NamedTuple):
    animation: Animation | None
    """
    Strip brightness when displaying the effect (0-1)
    """
    brightness: float | None = None
    """
    Duration in seconds before stopping the effect (s)
    """
    duration: float | None = None
    """
    Number of full iterations before stopping the effect
    """
    iterations: int | None = None


"""
This is the dictionary where the service callback looks up animation names.
It should get a function that builds the animation.
The function will be passed a Pixelbuf object, and an AnimationInfo object
and should return the Animation object.
"""
ANIMATION_LOOKUP: dict[str, Callable[[NeoPixelRing_SPI, AnimationInfo], Animation]] = {
    'base': lambda _, _1: None,

    '': lambda pixels, info: Solid(pixels, info.color),
    'solid': lambda pixels, info: Solid(pixels, info.color),

    'cycle': lambda pixels, info: ColorCycle(pixels, info.speed),
    'blink': lambda pixels, info: Blink(pixels, info.speed, info.color),
    'pulse': lambda pixels, info: Pulse(pixels, info.speed, info.color),
    'chase': lambda pixels, info: Chase(pixels, info.speed, info.color, info.size,
                                        spacing=CHASE_SPACING, reverse=info.reverse),
    'comet': lambda pixels, info: Comet(pixels, info.speed, info.color, tail_length=max(0, info.size - 1),
                                        reverse=info.reverse, ring=True),
    'rainbow': lambda pixels, info: Rainbow(pixels, info.speed, period=5, step=1 * (-1 if info.reverse else 1)),
    'rainbow_chase': lambda pixels, info: RainbowChase(pixels, info.speed, size=info.size,
                                                       spacing=CHASE_SPACING, reverse=info.reverse, step=8),
    'rainbow_comet': lambda pixels, info: RainbowComet(pixels, info.speed, tail_length=max(0, info.size - 1),
                                                       everse=info.reverse, ring=True),

    'channel_wrap': lambda pixels, info: ChannelWrapAnim(pixels, info.speed)
}


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

        self.animation_queue = PriorityQueue()
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
        self.animation_changed.set()

    def play_boot_anim(self) -> None:
        if not self.update_thread.is_alive():
            self.pixels.fill(0)
            boot_anim = ChannelWrapAnim(self.pixels, BOOT_ANIM_SPEED)
            run_anim_until_done(boot_anim)

    def set_callback(self, request: SetLedEffect.Request, response: SetLedEffect.Response) -> SetLedEffect.Response:
        effect = request.effect
        speed = ensure_non_zero(request.speed, DEFAULT_ANIM_SPEED)
        color = request.r, request.g, request.b
        size = ensure_non_zero(request.size, DEFAULT_ANIM_SIZE)
        info = AnimationInfo(speed, color, request.reverse, size)
        brightness = ensure_non_zero(request.brightness)

        duration = ensure_non_zero(request.duration)
        iterations = ensure_non_zero(request.iterations)

        priority = -request.priority

        if effect in ANIMATION_LOOKUP:
            if priority > 0:
                with self.animation_queue.mutex:
                    self.animation_queue.queue.clear()

            animation = ANIMATION_LOOKUP[effect](self.pixels, info)
            self.animation_queue.put((priority, AnimationEntry(animation, brightness, duration, iterations)))
            self.animation_changed.set()

            response.success = True
            response.message = 'Success'
        else:
            response.success = False
            response.message = f'Unknown effect: \"{request.effect}\"'

        return response

    def update_loop(self) -> None:
        iter_event = Event()

        priority: int | None = None

        start_time: float = -1.0
        animation: Animation | None = None
        brightness: float | None = None
        duration: float | None = None
        iterations: int | None = None

        self.animation_changed.set()
        while True:
            if self.animation_changed.is_set():
                self.animation_changed.clear()

                with self.animation_queue.mutex:
                    queue_list = list(self.animation_queue.queue)
                if priority is None or (len(queue_list) > 0 and min(queue_list)[0] <= priority):
                    start_time = time.time()
                    try:
                        priority, entry = self.animation_queue.get(block=False)
                    except Empty:
                        priority = None
                        entry = AnimationEntry(None)
                    animation, brightness, duration, iterations = entry

                    if iterations is not None:
                        animation.add_cycle_complete_receiver(lambda _: iter_event.set())

                    if brightness is not None:
                        self.pixels.brightness = brightness

                    self.pixels.fill(self.base_color)

            if animation is not None:
                animation.animate()

                if iter_event.is_set():
                    iter_event.clear()
                    if iterations is not None:
                        iterations -= 1
            else:
                self.pixels.fill(self.base_color)
                self.pixels.show()

            time_done = duration is not None and (time.time() - start_time) >= duration
            iter_done = iterations is not None and iterations <= 0

            if animation is None or isinstance(animation, Solid):
                self.animation_changed.wait(duration)
                priority = None
            elif time_done or iter_done:
                self.animation_changed.set()
                priority = None


def main(args=None):
    rclpy.init(args=args)
    led_strip_node = LEDStripNode()
    rclpy.spin(led_strip_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
