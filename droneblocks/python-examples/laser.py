import pigpio
from time import sleep

pi = pigpio.pi()

if not pi.connected: raise Exception('Cannot connect to pigpiod')

def gpio_write(pin, level):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, level)

while True:
    gpio_write(18, True)
    sleep(1)
    gpio_write(18, False)
    sleep(1)