from gpiozero import LED
from time import sleep

laser = LED(18)

while True:
    laser.on()
    sleep(1)
    laser.off()
    sleep(1)


"""
# Blink w/ PWM
from gpiozero import PWMLED
from time import sleep

laser = PWMLED(18)

while True:
    laser.value = 0
    sleep(1)
    laser.value = 1
    sleep(1)
"""