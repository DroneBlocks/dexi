# PX4 NSH Shell Access

PX4 serves its NuttX shell (`nsh`) over MAVLink. On DEXI, you can access the shell
from the companion computer over SSH without opening the airframe or attaching a
debug probe.

> **Don't use the shell in flight.** It runs as a low-priority task and blocking
> commands can starve it. Bench and ground testing only.

## Prerequisite: Connect DEXI to the internet

Before continuing, connect DEXI to an internet-connected Wi-Fi network. Follow the
DEXI networking guide to configure your home or school Wi-Fi:

[Configure Home/School Wi-Fi](https://github.com/DroneBlocks/dexi-networking#configure-homeschool-wifi)

Once connected, DEXI can usually be reached at `dexi-XXXX.local`, where `XXXX` is the
identifier in the drone's hostname.

## Connect to DEXI over SSH

`mavlink-router` exposes the flight controller's MAVLink stream on UDP port `14550`.
Nothing needs to be stopped—ROS 2, `mavlink2rest`, and QGroundControl can remain
running.

SSH into DEXI:

```bash
ssh dexi@dexi-XXXX.local
```

The default password is `dexi`.

Install the required Python packages. These packages are not included in DEXI OS by
default:

```bash
python3 -m pip install --break-system-packages pyserial
python3 -m pip install --break-system-packages pymavlink
```

Download the PX4 MAVLink shell script:

```bash
curl -sfL -o ~/mavlink_shell.py \
  https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/mavlink_shell.py
```

Start the shell:

```bash
python3 ~/mavlink_shell.py udpout:127.0.0.1:14550
```

You will land at an `nsh>` prompt. Exit with **Ctrl-D**.

The loopback address is correct because `mavlink_shell.py` is running directly on
DEXI and connects to `mavlink-router` locally.

## Checking it works

```text
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

Other useful commands include `listener <topic>`, `dmesg`, `top`, `uorb status`, and
`perf`.

## Troubleshooting

**The shell hangs with no output.** Use `udpout:`, not a bare address.
`mavlink-router`'s endpoints wait for the client to send first. Using
`127.0.0.1:14550` alone makes `pymavlink` bind and listen, so the router never learns
where to reply.

**The shell opens, but output goes missing.** PX4 keeps one shell session per MAVLink
link. If QGroundControl's *Analyze → MAVLink Console* is open on the same link, it can
take over the session. Close the MAVLink Console and reconnect.
