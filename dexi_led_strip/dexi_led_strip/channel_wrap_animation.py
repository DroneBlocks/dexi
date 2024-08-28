from adafruit_led_animation.animation import Animation
from adafruit_led_animation.color import calculate_intensity
from adafruit_led_animation.animation.comet import Comet

from dexi_led_strip.neopixel_ring_spi import MirroredPixelbufView


class ChannelWrapAnim(Animation):
    def __init__(self, pixel_object, speed: float, channel_count: int = 3) -> None:
        mirrored = MirroredPixelbufView(pixel_object)
        super().__init__(mirrored, speed, 0)

        self._led_count = len(mirrored)
        self._colors = [(0,) * i + (255,) + (0,) * (channel_count - i - 1) for i in range(channel_count)] + [(0,) * channel_count]
        self._comet = Comet(mirrored, speed=0.01, color=(255,) * channel_count, tail_length=2, reverse=True)
        self._comet.add_cycle_complete_receiver(self._anim_finished)

        self.add_cycle_complete_receiver(lambda _: self.reset())

        self._state = 0

    def _anim_finished(self, _) -> None:
        self.cycle_complete = True

    def draw(self) -> None:
        pixels = self.pixel_object
        index = self._state % self._led_count
        color_index = self._state // self._led_count

        if color_index < len(self._colors):
            pixels[index] = self._colors[color_index]
        else:
            #index = -index % self._led_count

            #pixels[max(0, index - 1):index] = self._white_colors[max(0, -index):]
            self._comet.animate()
        
        self._state += 1

    def reset(self) -> None:
        self._state = 0   
        self._comet.reset() 
