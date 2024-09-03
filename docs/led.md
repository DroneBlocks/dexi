# LED Setup

Note: it is imperative to set the following param in /boot/firmware/config.txt for LED control to work properly via SPI:

```
core_freq=500
core_freq_min=500
```

and reboot.

Per the readme here: https://github.com/jgarff/rpi_ws281x

## Install

```
sudo apt install python3-rpi.gpio
sudo pip3 install adafruit-circuitpython-neopixel
sudo pip3 install rpi_ws281x
```

## Command Line test

```
ros2 service call /dexi/led_service/set_led_pixel_color dexi_interfaces/srv/LEDPixelColor "{index: 0, r: 0, g: 255, b: 0}"
ros2 service call /dexi/led_service/set_led_ring_color dexi_interfaces/srv/LEDRingColor "{color: 'purple'}"
```

# OLD

Raspberry Pi 4 and Ubuntu 22 standalone test.

## Dependencies to build

```
sudo apt install cmake -y
```

From user's home directory:

```
git clone https://github.com/jgarff/rpi_ws281x
cd rpi_ws281x
mkdir build
cd build
cmake -D BUILD_SHARED=OFF -D BUILD_TEST=ON ..
cmake --build .
sudo make install
```

Make sure LED is wired up to 5V, GND, and GPIO 21 and run:

```
sudo ./test -x 34 -y 1 -g 21 -c
```

The command above specifies a single row strip of 34 LEDs connected to gpio 21.
