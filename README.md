# Preflight Checklist

Hardware validation and configuration scripts that run **on the robot**. These
are standalone operator tools — no ROS 2 dependency — that talk directly to the
hardware over SocketCAN, RTDE, and GPIO. Run them before an operational session
(validation) or when commissioning/tuning a drive (configuration).

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
  `ip link set can0 up type can bitrate 1000000`).
- The tuning and homing scripts write to drive NVM only when you confirm; values
  are otherwise volatile and revert on power cycle.
- These are operator tools, not services — they are meant to be run by hand and
  read interactively.
