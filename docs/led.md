# LED Setup

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
