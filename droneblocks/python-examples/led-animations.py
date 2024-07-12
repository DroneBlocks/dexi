import board
import neopixel

from adafruit_led_animation.animation.solid import Solid
from adafruit_led_animation.animation.colorcycle import ColorCycle
from adafruit_led_animation.animation.blink import Blink
from adafruit_led_animation.animation.comet import Comet
from adafruit_led_animation.animation.chase import Chase
from adafruit_led_animation.animation.pulse import Pulse
from adafruit_led_animation.animation.rainbow import Rainbow
from adafruit_led_animation.animation.rainbowsparkle import RainbowSparkle
from adafruit_led_animation.animation.rainbowcomet import RainbowComet
from adafruit_led_animation.sequence import AnimationSequence
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
    AQUA
)

# Update to match the pin connected to your NeoPixels
pixel_pin = board.D12
# Update to match the number of NeoPixels you have connected
pixel_num = 45

pixels = neopixel.NeoPixel(pixel_pin, pixel_num, brightness=0.5, auto_write=False)

solid = Solid(pixels, color=PINK)
blink = Blink(pixels, speed=0.5, color=JADE)
colorcycle = ColorCycle(pixels, speed=0.4, colors=[MAGENTA, ORANGE, TEAL])
chase = Chase(pixels, speed=0.1, color=WHITE, size=3, spacing=6)
chase2 = Chase(pixels, speed=0.1, color=RED, size=3, spacing=6)
chase3 = Chase(pixels, speed=0.1, color=YELLOW, size=3, spacing=6)
chase4 = Chase(pixels, speed=0.1, color=ORANGE, size=3, spacing=6)
chase5 = Chase(pixels, speed=0.1, color=GREEN, size=3, spacing=6)
comet = Comet(pixels, speed=0.01, color=PURPLE, tail_length=10, bounce=True)
comet2 = Comet(pixels, speed=0.01, color=CYAN, tail_length=10, bounce=True)
comet3 = Comet(pixels, speed=0.01, color=MAGENTA, tail_length=10, bounce=True)
comet4 = Comet(pixels, speed=0.01, color=GOLD, tail_length=10, bounce=True)
comet5 = Comet(pixels, speed=0.01, color=PINK, tail_length=10, bounce=True)
pulse = Pulse(pixels, speed=0.1, color=AQUA, period=3)
rainbow = Rainbow(pixels, speed=0.1, period=2)
rainbow_sparkle = RainbowSparkle(pixels, speed=0.1, num_sparkles=15)
rainbow_comet = RainbowComet(pixels, speed=0.1, tail_length=45)

animations = AnimationSequence(
    rainbow_comet,
    rainbow_sparkle,
    rainbow,
    solid,
    blink,
    colorcycle,
    chase,
    comet,
    pulse,
    comet2,
    chase2,
    rainbow_comet,
    comet3,
    chase3,
    comet4,
    rainbow_comet,
    chase4,
    advance_interval=15,
    auto_clear=True,
)

while True:
    animations.animate()