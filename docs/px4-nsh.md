# PX4 NSH Shell Access

PX4 serves its NuttX shell (`nsh`) over MAVLink, so any link that already carries
MAVLink to the flight controller can also carry a shell. On DEXI that means you can
reach `nsh` from the companion computer, from your laptop over the network, or over a
USB cable — without opening the airframe or attaching a debug probe.

> **Don't use the shell in flight.** It runs as a low-priority task and blocking
> commands can starve it. Bench and ground testing only.

## Connecting to the drone

The drone broadcasts its own Wi-Fi access point, named after its hostname
(`dexi-XXXX`). Join that network and the drone is at **`192.168.4.1`**.

If the drone has instead joined an existing network, use `dexi-XXXX.local` and
substitute that for `192.168.4.1` below.

## On the drone, via SSH

`mavlink-router` exposes the flight controller's MAVLink stream on UDP `14550`.
Nothing needs to be stopped — ROS 2, `mavlink2rest` and QGroundControl can stay up.

```bash
ssh dexi@192.168.4.1          # password: dexi

curl -sfL -o ~/mavlink_shell.py \
  https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/mavlink_shell.py

python3 ~/mavlink_shell.py udpout:127.0.0.1:14550
```

You land at an `nsh>` prompt. Exit with **Ctrl-D**.

The loopback address is correct here — `mavlink_shell.py` is running on the drone, so it
connects to the router locally. `pymavlink` is preinstalled on DEXI-OS.

## From your computer, over the network

`mavlink-router` binds `0.0.0.0`, so the same endpoint is reachable from any host that
can see the drone.

```bash
pip3 install --user pymavlink pyserial
python3 mavlink_shell.py udpout:192.168.4.1:14550
```

## Over USB, directly to the flight controller

The companion computer talks to the flight controller over UART, so the FC's USB-C port
is free. MAVLink starts on it automatically when you plug in; no parameter changes are
needed.

The connector is on the top side of the board, next to the SD card and the BOOT button.

```bash
python3 mavlink_shell.py /dev/tty.usbmodem1101      # macOS / Linux
python3 mavlink_shell.py COM9                       # Windows
```

## Checking it works

```
nsh> ver all
HW arch: DRONEBLOCKS_H743_AIO
PX4 version: 1.17.0
OS: NuttX, Release 11.0.0

nsh> commander status
INFO  [commander] Disarmed
INFO  [commander] navigation mode: Hold
INFO  [commander] in failsafe: no

nsh> param show SER_TEL1_BAUD
x   SER_TEL1_BAUD [890,1314] : 500000
```

Also useful: `listener <topic>`, `dmesg`, `top`, `uorb status`, `perf`.

## Troubleshooting

**The shell hangs with no output.** Use `udpout:`, not a bare address.
`mavlink-router`'s endpoints wait for the client to send first, so `127.0.0.1:14550`
alone makes pymavlink bind and listen, and the router never learns where to reply.

**The shell opens but output goes missing.** PX4 keeps one shell session per MAVLink
link. If QGroundControl's *Analyze → MAVLink Console* is open on the same link, it will
take over the session. Close it.

A shell opened over USB is a separate link from one opened over telemetry, so a
network-connected QGroundControl won't interfere with a USB shell.
