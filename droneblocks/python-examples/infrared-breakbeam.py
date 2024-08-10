import pigpio
import board
import neopixel

BEAM_PIN = 24
PIXEL_COUNT = 44
PIXEL_PIN = board.D12
PIXEL_ORDER = neopixel.GRB
PIXELS = neopixel.NeoPixel(PIXEL_PIN, PIXEL_COUNT, brightness=0.2, auto_write=False, pixel_order=PIXEL_ORDER)

pi = pigpio.pi()
if not pi.connected: raise Exception('Cannot connect to pigpiod')
pi.set_mode(BEAM_PIN, pigpio.INPUT)
pi.set_pull_up_down(BEAM_PIN , pigpio.PUD_UP)

def cbf(gpio, level, tick):

    if level == 1:
        toggle_led(True)
    else:
        toggle_led(False)

    print(gpio, level, tick)


def toggle_led(status):
    for n in range(PIXEL_COUNT):
        if (status):
            PIXELS[n] = (255, 255, 255)
        else:
            PIXELS[n] = (0, 0, 0)

    PIXELS.show()

toggle_led(False)

cb1 = pi.callback(BEAM_PIN, pigpio.EITHER_EDGE, cbf)

message = input('Press enter to quit\n')

cb1.cancel()