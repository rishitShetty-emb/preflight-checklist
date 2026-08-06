# Preflight Checklist

Hardware validation and configuration scripts that run **on the robot**. These
are standalone operator tools — no ROS 2 dependency — that talk directly to the
hardware over SocketCAN, RTDE, and GPIO. Run them before an operational session
(validation) or when commissioning/tuning a drive (configuration).

One exception: [`can-home-motor-usb`](#can-home-motor-usb) runs from a **laptop**
through a USB-CAN adapter instead of on-robot SocketCAN. See
[USB-CAN adapter setup](#usb-can-adapter-setup-laptop).

## Buses & node map

| Bus     | Interface | Nodes                                                        |
|---------|-----------|--------------------------------------------------------------|
| AMR     | `can0`    | 1 `wheel_f`, 2 `steering_f`, 3 `wheel_rl`, 4 `steering_rl`, 5 `wheel_rr`, 6 `steering_rr` |
| Liftkit | `can1`    | 7 lift motor, 8 tool node                                    |
| GPIO    | `gpiochip2` | controller compute I/O board                               |
| UR arm  | Ethernet  | dashboard `:29999`, primary `:30001`, RTDE `:30004`          |

Travel motors (drive wheels) are the odd nodes **1, 3, 5**; steering motors are
the even nodes **2, 4, 6**.

Most CAN scripts put nodes into **NMT Pre-Operational** first so SDO works and
the drives stay idle (no motion), and use expedited CANopen SDO
(read `0x40` / write `0x2F`/`0x2B`/`0x23`, abort `0x80`, ack `0x60`).

---

## `preflight_check`

**Full actuation preflight for the whole robot.** Python, standalone. The main
go/no-go check run before operating.

Sections (each can be skipped with a flag):

- **UR arm** — dashboard queries (robot mode, safety mode, remote-control),
  reachability of ports 30001/30004, and an RTDE session that measures loop
  frequency / latency / command-sync at 500 Hz and dumps a state snapshot.
- **Liftkit (`can1`)** — lift-motor statusword, tool-node state, PID + safety
  parameter dump, and a CAN timing test.
- **AMR (`can0`)** — statuswords for all six nodes, PID dump, steering homing
  (nodes 2/4/6 driven to position 0 via CiA 402), and a CAN timing test.
- **CAN timing test** — reprograms TPDO1/TPDO2 via SDO, then runs a 5 s, 500 Hz
  RPDO→SYNC→TPDO loop with drives left in Switch-On-Disabled (zero motion) and
  reports jitter, TPDO coverage, and SDO latency.
- **Tool electromechanical test** — interactive, always last. Detects the tool
  (sander / sprayer), then steps through bucket, valve/motor, speed levels,
  load cell, latches, and HMI buttons with operator Y/N confirmation. Restores
  all physical state on exit, even on Ctrl-C.

```
python3 preflight_check [--skip-ur] [--skip-liftkit] [--skip-amr] \
                        [--skip-tool] [--ur-ip 192.168.4.102]
```

Exit code is non-zero if any check failed (`PREFLIGHT FAILED — do not launch`).

---

## `amr_pid_tune`

**Interactive PID configurator for the three AMR travel motors** (`can0`, nodes
1/3/5). Python, standalone. You enter each gain once and it is written to **all
three nodes** so they stay identical.

Walks six gains — Kvp (`0x60F9:01`), Kvi (`0x60F9:03`), Kvi_Sum_Limit
(`0x60F9:0C`), Speed_Fb_N (`0x60F9:05`), Kpp (`0x60FB:01`), Kvff (`0x2FF0:1A`).
For each: reads the current value from every node and shows them side-by-side
(flagging any that are not already uniform), prompts once, writes to all three,
then reads back each node with a per-node ✓ / mismatch / fail marker. Optional
save to non-volatile memory at the end. Write width is auto-detected from the
first responding node (falls back to INT32).

```
python3 amr_pid_tune [--iface can0] [--nodes 1 3 5]
```

Steering nodes (2/4/6) are intentionally left untouched. Modelled on
`liftkit_pid_tune`.

---

## `liftkit_pid_tune`

**Interactive PID configurator for the liftkit drive** (`can1`, node 7). Python,
standalone. The single-node original that `amr_pid_tune` is based on.

Same six gains as `amr_pid_tune`. For each gain: reads the current value,
prompts for a new one (Enter keeps it), writes, and reads back to confirm, then
prints the relevant design-constraint reminder (filter BW vs. velocity-loop BW,
position-loop BW vs. velocity-loop BW). Optional non-volatile save at the end.

```
python3 liftkit_pid_tune [--iface can1] [--node 7]
```

---

## `can-home-motors`

**Auto-homing for multiple servo motors** over CANopen SDO. Python, standalone.
Each motor runs in its own thread with its own bus socket.

For each configured motor (default `Front`=2, `Left`=4, `Right`=6 on `can0`):
reboots the driver, sweeps clockwise until a following-error/limit, sweeps
counter-clockwise until a limit, computes the midpoint, moves there, resets the
encoder, and saves the resulting home value to non-volatile memory. Ctrl-C stops
all motors safely.

```
python3 can-home-motors [-i can0] [-b socketcan] [-m Front Left ...]
```

Before the homing state machine runs on a node, `0x6098:00` Homing_Method is
verified and written to **35** if it differs (applies immediately — no
power-cycle needed, unlike the pre-flight config ODs).

---

## `can-home-motor-usb`

**Single-node auto-homing from a laptop over a USB-CAN adapter.** Python,
standalone. Same homing logic and same core state machine as `can-home-motors`,
with two differences: it talks to a **CANalyst-II / USBCAN-2A** USB adapter
instead of SocketCAN, and it homes exactly **one** node per run.

Same three modes (`full` sweep→midpoint, `align` jog, `blind` home-in-place) and
the same FSM:

```
0x6098=35 → 0x6060=6 → CW 0x06 → 0x2690=10 → CW 0x0F → CW 0x1F → 0x1010:01="save"
```

```
python can-home-motor-usb.py --node 2 --mode blind
python can-home-motor-usb.py                        # menus for mode + node
```

| Flag | Default | Meaning |
|---|---|---|
| `--node` / `-n` | prompts | CANopen node ID, 1–127 |
| `--mode` / `-M` | prompts | `full` \| `align` \| `blind` |
| `--channel` / `-c` | `0` | adapter CAN port: `0` = CAN1, `1` = CAN2 |
| `--bitrate` | `1000000` | bit/s (AMR bus is 1 Mbit/s) |
| `--bustype` / `-b` | `canalystii` | python-can backend; `socketcan` to run on the robot |
| `-y` | off | skip the non-steer-node confirmation |
| `--skip-dir-check` | off | skip the pre-flight OD check |

Behaviour worth knowing:

- **Fail-fast probe.** Reads `0x1000:00` (Device Type) before writing anything. If
  the node is silent it drops the receive filter and lists any bus traffic, which
  separates "bus/wiring/bitrate dead" from "bus fine, wrong node ID".
- **Pre-flight OD check covers only the target node** — so a bench setup with a
  single drive connected works. Nodes with no expectation on record skip
  `0x607E`/`0x6410`; non-steer nodes skip `0x6099:03`/`:05`.
- **Ctrl-C sends Shutdown** (`0x6040=0x0006`) to drop torque before exiting.
- **`full` mode sweeps for mechanical hard stops.** The steer axes (2/4/6) have
  them; the travel wheels (1/3/5) do not, so on a travel node each sweep runs to
  its 300 s timeout while the wheel turns. Pick the mode to match the axis.
- Every successful run writes the new encoder zero to **NVM**, replacing the
  previous home reference.

---

## USB-CAN adapter setup (laptop)

Setup for `can-home-motor-usb`. The adapter is a **Zhuhai Chuangxin
USBCAN-2A / CANalyst-II** (`VID_04D8` / `PID_0053`), two CAN channels.

### 1. Install the driver

Run either installer (Windows):

```
USB-CAN-B-Driver-Setup(V1.40).exe
USB_CAN TOOL/driver/dpinst64.exe      # or dpinst32.exe on 32-bit
```

This binds the device to **WinUSB** via `mchpwinusb.inf`.

> The adapter is **not** a virtual COM port. No `COMx` will ever appear in Device
> Manager, and that is correct — frames travel as raw USB bulk transfers. Don't go
> looking for a serial port.

Confirm it enumerated (PowerShell):

```powershell
Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match 'VID_04D8' } |
  Select-Object Status, Class, FriendlyName, InstanceId
```

Expected — `Status: OK`, class `CustomUSBDevices`:

```
OK   CustomUSBDevices   WinUSB Device   USB\VID_04D8&PID_0053\...
```

### 2. Install the Python packages

```
pip install python-can canalystii libusb-package
```

`libusb-package` ships `libusb-1.0.dll`, which pyusb needs. Without a libusb on
the DLL search path the bus fails to open with **`No backend available`** even
though the adapter is fine. The script puts the bundled DLL on `PATH` at import
time, so installing this package is all that's required.

The vendor `ControlCAN.dll` is **not** used: it is 32-bit only and cannot be
loaded from 64-bit Python. The `canalystii` backend drives the same WinUSB device
in pure Python.

### 3. Wire the CAN side

- CAN-H → CAN-H, CAN-L → CAN-L (not swapped), and **share GND** with the drives.
- 120 Ω termination at both ends of the bus.
- Note which physical connector you use: `CAN1` → `--channel 0`, `CAN2` →
  `--channel 1`. The robot-side names (`can0`, `can1`) do **not** map to the
  adapter — the channel is whichever port you plugged into.
- Match `--bitrate` to the bus. AMR (`can0`) is **1 Mbit/s**.

### 4. Close the vendor GUI

`USB_CAN_Tool.exe` claims the adapter exclusively. If it's open, the script fails
with an `Access denied` / backend error. Close it first.

### 5. Verify communication

```
python can-home-motor-usb.py --node 1 --mode blind
```

A working bus prints the probe result before anything is written:

```
[wheel_f] Probing node 1...
[wheel_f] Node 1 responded: 0x1000:00 (Device Type) = 0x00020192
```

`0x00020192` decodes as `0x0192` = 402 → CiA-402 drive profile, `0x0002` = servo
drive. That is a confirmed SDO round trip.

> Stop here if you only wanted to confirm the link — the command above **continues
> into homing** and writes a new encoder zero to NVM. Ctrl-C at the probe line if
> that isn't what you want.

### Troubleshooting

| Symptom | Cause |
|---|---|
| `No backend available` | pyusb found no libusb → `pip install libusb-package` |
| `No Canalyst-II USB device found` | adapter unplugged, or driver not installed (step 1) |
| `Access denied (insufficient permissions)` | another process holds it — usually `USB_CAN_Tool.exe` |
| Node silent, **no** bus traffic listed | drives unpowered, wrong `--channel`, wiring/termination, or wrong `--bitrate` |
| Node silent, but other IDs listed | bus is fine — wrong node ID, or that one drive is off |
| No `COMx` in Device Manager | expected; this is a WinUSB device, not a serial port |

---

## `gpio_preflight_check`

**Controller-compute GPIO board preflight** on `gpiochip2`. Bash. Must run as
root. Validates every line on the board before an operational session.

Pulses the board-init line, drives all outputs to a safe/idle default, checks
input default states, then walks interactive tests for the tool limit switch,
contactor feedback, E-Stop (press + release), and the ASB outputs
(error-trigger pulse, recovery on/off). Prints a PASS/FAIL/SKIP summary and
exits non-zero on any failure. The header documents the full `gpiochip2` pin map
and active-high/active-low polarity of each line.

```
sudo gpio_preflight_check
```

---

## Notes

- CAN scripts assume the relevant interface is already up (e.g.
  `ip link set can0 up type can bitrate 1000000`). `can-home-motor-usb` is the
  exception — it configures the adapter itself from `--bitrate`.
- The `*_pid_tune` scripts write to drive NVM only when you confirm; values are
  otherwise volatile and revert on power cycle. The homing scripts are different:
  the save to NVM is part of the homing state machine and is **not** prompted.
- These are operator tools, not services — they are meant to be run by hand and
  read interactively.
