# GPIO

### Install

From here: https://abyz.me.uk/rpi/pigpio/download.html do the following:

```
cd
git clone https://github.com/joan2937/pigpio
cd pigpio
make
sudo make install
sudo pigpiod
```

### Run

In the dexi/droneblocks/python-examples folder run the test connected to GPIO 18:

```
python3 laser.py
```

## Service

```
sudo cp ~/ros2_ws/src/dexi/systemd/pigpiod.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable pigpiod.service
```
