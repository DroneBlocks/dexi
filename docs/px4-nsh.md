# PX4 NSH Shell Access

The NuttX shell (`nsh`) running on the DEXI flight controller can be reached without
opening the airframe or attaching a debug probe. PX4 serves the shell over MAVLink
itself, using `SERIAL_CONTROL` messages, so any link that already carries MAVLink to
the FC will also carry a shell.

That means three routes, all equivalent once you're at the `nsh>` prompt:

| Route | Where you run it | Needs |
|---|---|---|
| Over the companion computer | SSH into the Pi | nothing extra — `pymavlink` ships on DEXI-OS |
| Over the network | Your laptop | `pip install pymavlink` |
| Over USB | Your laptop | `pip install pymavlink`, or MATLAB's `HelperPX4` |

> **Do not use the shell in flight.** It runs as a low-priority task and blocking
> commands can starve it. Bench and ground testing only.

---

## Connecting to the drone

The drone broadcasts its own Wi-Fi access point, named after its hostname
(`dexi-XXXX`). Join that network and the drone is always reachable at **`192.168.4.1`**
— a fixed address, unaffected by DHCP.

If instead the drone has joined an existing network, use `dexi-XXXX.local` and
substitute that for `192.168.4.1` throughout.

## Route 1 — On the drone, via SSH

The onboard `mavlink-router` already exposes the FC's MAVLink stream on UDP `14550`.
Nothing needs to be stopped: ROS 2, `mavlink2rest` and QGroundControl can all stay up.

```bash
ssh dexi@192.168.4.1          # password: dexi

curl -sfL -o ~/mavlink_shell.py \
  https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/mavlink_shell.py

python3 ~/mavlink_shell.py udpout:127.0.0.1:14550
```

You land at an `nsh>` prompt. Exit with **Ctrl-D**.

Note the loopback address: `mavlink_shell.py` is running *on* the drone, so it connects
to the router locally. `pymavlink` is preinstalled on DEXI-OS, so there is no `pip` step.

## Route 2 — From your computer, over the network

`mavlink-router` binds `0.0.0.0`, so the same UDP endpoint is reachable from any host
that can see the drone — including over its access point:

```bash
pip3 install --user pymavlink pyserial
python3 mavlink_shell.py udpout:192.168.4.1:14550
```

## Route 3 — Over USB, directly to the flight controller

On DEXI-3 the companion computer reaches the FC over UART (TELEM1 @ 500000 baud), so
the FC's USB-C port is free. MAVLink starts on it automatically — the board sets
`CONFIG_DRIVERS_CDCACM_AUTOSTART=y`, and `SYS_USB_AUTO` defaults to `2` (MAVLink) with
`USB_MAV_MODE=2` (onboard), which runs `mavlink start -d /dev/ttyACM0 -m onboard` the
moment you plug in. No parameter changes are required.

The USB-C connector is on the top (STM32) side of the board, next to the SD card and
the BOOT button.

```bash
python3 mavlink_shell.py /dev/tty.usbmodem1101      # macOS / Linux
python3 mavlink_shell.py COM9                       # Windows
```

---

## Verifying it works

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

Other useful commands: `listener <topic>`, `dmesg`, `top`, `uorb status`, `perf`.

---

## Gotchas

**Use `udpout:`, not a bare address.** `mavlink-router`'s endpoints are `Mode=Server`,
meaning they wait for a client to send first. A bare `127.0.0.1:14550` makes pymavlink
bind and listen, the router never learns where to reply, and the shell hangs with no
error.

**One shell client per MAVLink instance.** PX4 tracks a single target for the shell
session (`shell->setTargetID(...)`, last writer wins). If QGroundControl's *Analyze →
MAVLink Console* is open on the same link, it will steal the session. Close it first.

Shells are per-instance, though: a shell opened over USB is independent of one opened
over TELEM1, so a network-connected QGC does not conflict with a USB shell.

**Source system IDs do not need to be unique across endpoints.** `mavlink_shell.py`
heartbeats as sysid 255, the same as QGC and `mavlink2rest`. This is fine as long as
they sit on different router endpoints — `mavlink-router` only refuses to forward a
sysid back through the endpoint it learned it from.

---

## MATLAB / MathWorks UAV Toolbox

MathWorks' `HelperPX4` class is a serial MAVLink client using the same `SERIAL_CONTROL`
mechanism — their docs state plainly that *"NSH is accessed using MAVLink."* It works
against DEXI-3 unmodified, over USB (Route 3):

```matlab
shellObj = HelperPX4('COM9');
[resp, status] = shellObj.system('ver all');
disp(resp)
shellObj.getFile('/fs/microsd/log/...')   % MAVLink FTP
delete(shellObj);
```

Two notes:

- The *"Enabling MAVLink in PX4 Over USB"* step in their documentation **does not
  apply**. That page describes firmware built by the support package, where MAVLink
  ships disabled. DEXI firmware has it enabled by default.
- `HelperPX4` accepts only a serial port, and MATLAB for Linux is x86-64 only, so it
  cannot run on the DEXI companion computer (aarch64). Use Route 3 from a laptop, or
  Route 1 with `mavlink_shell.py`.

If you intend to **deploy a Simulink model** rather than just read the shell, note that
DEXI-3 uses a custom board (`droneblocks/h743-aio`, PX4 board ID 1240) that is not on
the support package's supported hardware list. Use *"My board is not listed here"* in
Hardware Setup and point it at that CMake target. Any firmware you flash must keep
`SER_TEL1_BAUD = 500000`, or the companion computer loses its link to the FC.

---

## How it works

`mavlink_shell.py` sends `SERIAL_CONTROL` messages with
`device = SERIAL_CONTROL_DEV_SHELL` and the `RESPOND | EXCLUSIVE` flags. PX4 handles
them in `src/modules/mavlink/mavlink_receiver.cpp`, forwarding bytes into a NuttX shell
task spawned by `mavlink_shell.cpp` and streaming the output back in reply messages.

`mavlink_shell.cpp` is compiled into the mavlink module unconditionally — it is only
excluded on `CONSTRAINED_FLASH` boards, which the STM32H743 is not.
