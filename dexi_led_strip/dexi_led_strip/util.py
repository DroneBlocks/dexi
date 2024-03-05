from threading import Event as _Event

from busio import SPI as _SPI
from board import SPI as SPI0
from adafruit_led_animation.animation import Animation as _Animation


# Configuration for the second SPI buss on the RPI4
SPI1 = lambda: _SPI(21, 20, 19)

# Easy way to get the bus object by port ID
SPI = [SPI0, SPI1]


def run_anim_until_done(anim: _Animation) -> None:
    event = _Event()
    anim.add_cycle_complete_receiver(lambda _: event.set())
    while not event.is_set():
        anim.animate()
