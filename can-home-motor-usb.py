#!/usr/bin/env python3
"""
CANopen Steer-Motor Auto-Homing — SINGLE NODE, USB-CAN adapter edition
----------------------------------------------------------------------
Same homing logic as `can-home-motors`, but:

  * runs from a laptop through a USB-CAN adapter (CANalyst-II / USBCAN-2A,
    VID 0x04D8 PID 0x0053 — the one in ../USB_CAN TOOL) instead of Linux
    SocketCAN, and
  * homes exactly ONE node, which you specify with --node.

Three homing options (interactive menu, or --mode flag):

  1) Full homing    Sweep both hard limits, drive to midpoint (a+b)/2, then home.
  2) Assisted align Confirm alignment or jog by +/- counts, then home.
  3) Blind homing   Home at the current position (no sweep, no jog).

All options run the same core homing state machine, which first verifies (and writes,
if needed) 0x6098:00 Homing_Method = 35:
  0x6098=35 -> mode 6 (Homing) -> CW 0x06 -> encoder reset (0x2690=10)
            -> CW 0x0F -> CW 0x1F -> save.

See docs/specs/2026-08-05-can-home-motors-redesign-design.md.


Setup on the laptop
-------------------
  1. Install the adapter driver: `USB-CAN-B-Driver-Setup(V1.40).exe`, or
     `USB_CAN TOOL/driver/dpinst64.exe`. It binds the device to WinUSB
     (mchpwinusb.inf), which is what the python backend below needs.
  2. pip install python-can canalystii libusb-package
     (libusb-package ships libusb-1.0.dll; without a libusb on the DLL search
     path pyusb fails with "No backend available". The bootstrap below finds it.)
  3. Close USB_CAN_Tool.exe — it holds the device exclusively.
  4. Wire CAN-H / CAN-L / GND to the drive bus, 120 ohm termination present,
     and make sure --bitrate matches the bus (AMR can0 is 1 Mbit/s).

  python can-home-motor-usb.py --node 2 --mode blind

Note on ControlCAN.dll: the vendor DLL shipped in `USB_CAN TOOL/` is 32-bit
only, so it cannot be ctypes-loaded from 64-bit Python. This script therefore
talks to the same adapter through python-can's pure-python `canalystii`
backend, which drives the identical WinUSB device. Any other python-can
backend still works via --bustype (e.g. `--bustype socketcan --channel can0`
if you ever run this on the robot).

Linux laptop with a Waveshare USB-CAN Analyzer (a DIFFERENT adapter):
  The Waveshare "USB-CAN Analyzer" / USB-CAN-A is a CH340 USB-serial device
  (`lsusb` shows `1a86:7523 QinHeng CH340`) -- not the CANalyst-II above, and
  not a SocketCAN netdev. On Linux the driver you need is the in-kernel CH340/
  CH341 usb-serial module (`ch341`); modern kernels autoload it and the adapter
  appears as /dev/ttyUSB0 (check `dmesg | grep -i ch341`). If it does not show
  up, install WCH's ch34x driver (CH341SER_LINUX). Add your user to the
  `dialout` group for /dev/ttyUSB0 access. It speaks a serial CAN protocol, so
  drive it through python-can's `seeedstudio` backend, e.g.:
      python can-home-motor-usb.py --node 2 --mode blind \
          --bustype seeedstudio --channel /dev/ttyUSB0 --bitrate 1000000
  This will NOT work through the canalystii backend (that is the CANalyst-II, a
  different device) nor socketcan (no can0). If a given Waveshare unit is not
  seeedstudio frame-compatible, use Waveshare's own Python demo instead.

Differences vs `can-home-motors`, all deliberate:
  * one node per run, no threads (the original ran one thread + bus per motor);
  * the pre-flight OD check covers only the target node;
  * Ctrl-C sends Shutdown (controlword 0x0006) to the node before exiting, so a
    laptop-driven axis mid-sweep drops torque instead of coasting into a stop;
  * the "move to midpoint" wait has a timeout (the original could hang forever);
  * 0x6098:00 Homing_Method is verified/set to 35 right before the FSM;
  * a fail-fast SDO probe of the target node runs before anything is written.

No node type is blocked. `full` sweeps for mechanical hard stops, which the steer
axes have and the travel wheels (1/3/5) do not — on a travel node the sweep has
nothing to detect and will turn the wheel until the 300 s per-sweep timeout.
Choose the mode to match the axis.
"""

import argparse
import os
import sys
import threading
import time

# pyusb (used by the canalystii backend) locates libusb-1.0.dll via the PATH.
# If the libusb-package wheel is installed, put its bundled DLL on the PATH here,
# BEFORE python-can imports pyusb — otherwise opening the bus dies with
# "No backend available" even though the adapter is plugged in and working.
try:
    import libusb_package
    os.environ["PATH"] = (os.path.dirname(libusb_package.__file__)
                          + os.pathsep + os.environ.get("PATH", ""))
except ImportError:
    pass

import can

# Progress output contains em-dashes. When stdout is redirected to a file on
# Windows it defaults to cp1252, where a print() would raise mid-sweep — with the
# axis still driving into a hard stop. Never let logging kill the run.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

# === CONFIGURATION ===
# Known node IDs on the robot, for labelling only. --node accepts any 1..127.
NODE_NAMES = {
    1: "wheel_f",
    2: "steering_f",     # "Front" steer
    3: "wheel_rl",
    4: "steering_rl",    # "Left" steer
    5: "wheel_rr",
    6: "steering_rr",    # "Right" steer
    7: "liftkit",
    8: "tool",
}

CAN_BUSTYPE       = "canalystii"            # python-can backend for the USB-CAN adapter
CAN_CHANNEL       = "0"                     # adapter CAN port: 0 = CAN1, 1 = CAN2
CAN_BITRATE       = 1_000_000               # AMR bus runs at 1 Mbit/s
PROFILE_SPEED     = 200
PROFILE_SPEED_ENC = 17500 * PROFILE_SPEED   # encoder units per RPM
TAR_POS_CW        =  20000000               # far target: drive through to the CW hard stop
TAR_POS_CCW       = -20000000               # far target: drive through to the CCW hard stop
LIMIT_STABILIZE_SEC = 1.0                   # brief settle at a hard limit before reading pos
JOG_STEP           = 10000                  # default +/- jog increment (encoder counts)
MIDPOINT_TIMEOUT   = 120                    # s to wait for the midpoint move to finish

# === DIRECTION / HOMING OD PRE-FLIGHT ===
# Verified/auto-corrected before homing so the driver moves in the expected frame.
INVERT_DIR_INDEX       = 0x607E   # sub 0x00 — logical positive direction
INVERT_DIR_SUB         = 0x00
INVERT_DIR_MOTOR_INDEX = 0x6410   # sub 0x13 — motor-level direction inversion
INVERT_DIR_MOTOR_SUB   = 0x13

# 0x607E:00 Invert_Dir — 1 = CW positive, 0 = CCW positive. Only node 1 is reversed.
INVERT_DIR_EXPECTED       = {1: 1, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}
# 0x6410:13 Invert_Dir_Motor — every node inverted.
INVERT_DIR_MOTOR_EXPECTED = {1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1}

STEER_NODES = (2, 4, 6)

# 0x6098:00 Homing_Method — must be 35 before the homing FSM runs. Unlike the
# config ODs above this takes effect immediately, so a correction here does NOT
# need a power-cycle: we fix it inline and carry on.
HOMING_METHOD_INDEX    = 0x6098
HOMING_METHOD_SUB      = 0x00
HOMING_METHOD_EXPECTED = 35

# 0x6099:03 Homing_Power_On — must be 2 on the steer nodes.
HOMING_6099_POWERON_INDEX    = 0x6099
HOMING_6099_POWERON_SUB      = 0x03
HOMING_6099_POWERON_EXPECTED = 2

# 0x6099:05 Home_Offset_Mode — must be 1 on the steer nodes.
HOMING_6099_OFFMODE_INDEX    = 0x6099
HOMING_6099_OFFMODE_SUB      = 0x05
HOMING_6099_OFFMODE_EXPECTED = 1

# CiA-402 operation modes
MODE_PROFILE_POSITION = 1
MODE_HOMING           = 6

# Statusword bits
SW_OP_ENABLED    = 0x0004   # bit 2
SW_FAULT         = 0x0008   # bit 3
SW_FOLLOW_ERROR  = 0x2000   # bit 13

home_value = None
stop_event = threading.Event()

# CANopen SDO COB-IDs
SDO_TX_BASE = 0x600   # client → server (our request)
SDO_RX_BASE = 0x580   # server → client (drive response)

SDO_READ_RESPONSES = (0x43, 0x4B, 0x4F)   # 4-byte / 2-byte / 1-byte upload responses


# ── Bus plumbing (USB-CAN adapter) ─────────────────────────────────────────────

def open_bus(bustype, channel, bitrate, node_id=None):
    """
    Open the USB-CAN adapter (or any python-can backend named by --bustype).

    canalystii takes an integer channel and needs an explicit bitrate — unlike
    socketcan, where the bitrate is set by `ip link` outside the script.
    When node_id is given, a receive filter is installed for that node's SDO
    response COB-ID so a busy bus (heartbeats, TPDOs) can't crowd out replies.
    """
    kwargs = {"interface": bustype}
    if bustype == "socketcan":
        kwargs["channel"] = str(channel)
    else:
        try:
            kwargs["channel"] = int(channel)
        except ValueError:
            kwargs["channel"] = channel
        kwargs["bitrate"] = int(bitrate)

    bus = can.interface.Bus(**kwargs)
    if node_id is not None:
        bus.set_filters([{"can_id": SDO_RX_BASE + node_id,
                          "can_mask": 0x7FF,
                          "extended": False}])
    return bus


def diagnose_silence(bus, node_id):
    """
    The target node did not answer. Drop the receive filter and listen for ANY
    traffic, which separates the two very different root causes:
      * nothing at all  -> bus/adapter side: power, wiring, termination, bitrate,
                           or the adapter is on the wrong CAN port (--channel)
      * other IDs seen  -> bus is fine, the node ID is wrong or that drive is off
    """
    print("  Listening 2 s for any bus traffic (receive filter off)...")
    try:
        bus.set_filters(None)
        seen = {}
        deadline = time.time() + 2.0
        while time.time() < deadline:
            frame = bus.recv(timeout=max(0.05, deadline - time.time()))
            if frame is not None:
                seen[frame.arbitration_id] = seen.get(frame.arbitration_id, 0) + 1
    finally:
        bus.set_filters([{"can_id": SDO_RX_BASE + node_id,
                          "can_mask": 0x7FF, "extended": False}])

    if not seen:
        print("  No CAN traffic at all. The bus side is not alive:")
        print("    - drive powered? (a powered CANopen node emits heartbeats)")
        print("    - adapter on the CAN port you wired? try the other --channel")
        print("    - CAN-H/CAN-L not swapped, GND shared, 120 ohm termination present")
        print("    - --bitrate matches the bus")
        return
    print(f"  Bus IS alive — {sum(seen.values())} frames, {len(seen)} distinct IDs:")
    for arb_id, count in sorted(seen.items()):
        hint = ""
        if 0x700 <= arb_id <= 0x77F:
            hint = f"  <- heartbeat from node {arb_id - 0x700}"
        elif 0x580 <= arb_id <= 0x5FF:
            hint = f"  <- SDO response from node {arb_id - 0x580}"
        print(f"    0x{arb_id:03X} x{count}{hint}")
    print(f"  ...but node {node_id} stayed silent — check the node ID / that drive's power.")


def probe_node(bus, node_id, name):
    """
    Confirm the target node answers SDO before writing anything to it.

    Without this, an absent node is only noticed later as a vague "could not reach
    a clean state" after a series of blind controlword writes — and not at all when
    --skip-dir-check is used. Reads 0x1000:00 (Device Type, mandatory in CANopen),
    falling back to 0x6041:00 (Statusword).
    """
    print(f"[{name}] Probing node {node_id}...")
    for index, sub, label in ((0x1000, 0x00, "Device Type"),
                              (0x6041, 0x00, "Statusword")):
        value = decode_response(send_read_telegram(bus, node_id, index, sub))
        if value is not None:
            print(f"[{name}] Node {node_id} responded: "
                  f"0x{index:04X}:{sub:02X} ({label}) = 0x{value:08X}")
            return True
    print(f"[{name}] Node {node_id} did not respond to SDO.")
    diagnose_silence(bus, node_id)
    return False


def emergency_halt(bus, node_id, name):
    """Drop the power stage (Shutdown, controlword 0x0006). Best-effort."""
    try:
        send_write_telegram(bus, node_id, 0x6040, 0x00, 0x06, 2)
        print(f"[{name}] Shutdown sent (controlword 0x0006) — power stage disabled.")
    except Exception as exc:   # noqa: BLE001 — we are already on the way out
        print(f"[{name}] WARN: could not send Shutdown: {exc}")


# ── SDO helpers ────────────────────────────────────────────────────────────────

def _sdo_payload_write(index, subindex, value, size_bytes):
    """Build an 8-byte SDO download payload."""
    index_lo = index & 0xFF
    index_hi = (index >> 8) & 0xFF
    if size_bytes == 1:
        cmd  = 0x2F
        data = list((value & 0xFF).to_bytes(1, 'little')) + [0x00, 0x00, 0x00]
    elif size_bytes == 2:
        cmd  = 0x2B
        data = list((value & 0xFFFF).to_bytes(2, 'little')) + [0x00, 0x00]
    elif size_bytes == 4:
        cmd  = 0x23
        data = list((value & 0xFFFFFFFF).to_bytes(4, 'little'))
    else:
        raise ValueError(f"Unsupported SDO size: {size_bytes}")
    return [cmd, index_lo, index_hi, subindex] + data


def send_write_telegram(bus, node_id, index, subindex, value, size_bytes):
    payload = _sdo_payload_write(index, subindex, value, size_bytes)
    msg = can.Message(arbitration_id=SDO_TX_BASE + node_id,
                      data=payload, is_extended_id=False)
    bus.send(msg)
    time.sleep(0.05)


def send_read_telegram(bus, node_id, index, subindex, timeout=1.0, retries=3):
    """Send an SDO upload request and return the response Message (or None)."""
    index_lo    = index & 0xFF
    index_hi    = (index >> 8) & 0xFF
    request     = [0x40, index_lo, index_hi, subindex, 0x00, 0x00, 0x00, 0x00]
    response_id = SDO_RX_BASE + node_id

    for _ in range(retries):
        msg = can.Message(arbitration_id=SDO_TX_BASE + node_id,
                          data=request, is_extended_id=False)
        bus.send(msg)
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            frame = bus.recv(timeout=remaining)
            if frame is None:
                break
            if (frame.arbitration_id == response_id
                    and len(frame.data) >= 8
                    and frame.data[0] in SDO_READ_RESPONSES):
                return frame
        time.sleep(0.02)
    return None


def decode_response(response, signed=False):
    if response is None:
        return None
    return int.from_bytes(response.data[4:8], byteorder='little', signed=signed)


# ── Drive telemetry ──────────────────────────────────────────────────────────────

def get_actual_position(bus, node_id):
    return decode_response(send_read_telegram(bus, node_id, 0x6064, 0x00), signed=True)


def get_error_code(bus, node_id):
    return decode_response(send_read_telegram(bus, node_id, 0x603F, 0x00))


def get_status_word(bus, node_id):
    return decode_response(send_read_telegram(bus, node_id, 0x6041, 0x00))


# ── Drive control primitives ───────────────────────────────────────────────────

def encoder_data_reset(bus, node_id):
    """Zero the encoder / multi-turn data at the current position (0x2690:00 = 10)."""
    send_write_telegram(bus, node_id, 0x2690, 0x00, 10, 1)


def clear_fault(bus, node_id):
    """Fault-reset rising edge on controlword bit 7, then back to Shutdown."""
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x80, 2)
    time.sleep(0.2)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x06, 2)
    time.sleep(0.2)


def reboot_driver(bus, node_id):
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x06, 2)
    time.sleep(0.2)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x07, 2)
    time.sleep(0.2)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x00, 2)
    time.sleep(1)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x06, 2)
    time.sleep(1)


def move_to_target(bus, node_id, pos):
    """Absolute move in Profile Position mode (mode 1) to an encoder position."""
    send_write_telegram(bus, node_id, 0x6060, 0x00, MODE_PROFILE_POSITION, 1)  # Profile Position
    send_write_telegram(bus, node_id, 0x6081, 0x00, PROFILE_SPEED_ENC, 4)      # Profile velocity
    send_write_telegram(bus, node_id, 0x607A, 0x00, pos, 4)                    # Target position
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x2F, 2)                   # Enable + new setpoint
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x3F, 2)                   # Execute move


def ensure_clean_enabled(bus, node_id, name, timeout=10):
    """
    Guarantee the drive starts from a clean, fault-free, ENABLED state BEFORE any sweep/jog.

    Fixes the startup bug: if the drive was already faulted when the script started, a stale
    fault / following-error bit would otherwise be read by the sweep as "hit the hard stop"
    without the axis ever moving. We clear the fault and confirm error code 0 + fault bit
    clear first, then enable.

    Returns True once the drive is enabled and clean, False on timeout.
    """
    print(f"[{name}] Ensuring clean enabled state...")
    reboot_driver(bus, node_id)
    clear_fault(bus, node_id)

    deadline = time.time() + timeout
    while not stop_event.is_set():
        err = get_error_code(bus, node_id)
        sw  = get_status_word(bus, node_id)
        if err is not None and sw is not None:
            faulted = bool(sw & SW_FAULT)
            if err == 0 and not faulted:
                break
            # still dirty — re-issue a fault reset edge
            clear_fault(bus, node_id)
        if time.time() > deadline:
            print(f"[{name}] WARN: could not reach a clean state "
                  f"(err={err} sw=0x{(sw or 0) & 0xFFFF:04X}) — aborting node")
            return False
        time.sleep(0.3)

    # Enable: Shutdown -> Switch On -> Enable Operation
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x06, 2)
    time.sleep(0.2)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x07, 2)
    time.sleep(0.2)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x0F, 2)
    time.sleep(0.2)

    sw = get_status_word(bus, node_id)
    if sw is not None and (sw & SW_OP_ENABLED):
        print(f"[{name}] Clean & enabled (SW=0x{sw & 0xFFFF:04X})")
        return True
    print(f"[{name}] WARN: not Operation-Enabled after enable (SW=0x{(sw or 0) & 0xFFFF:04X})")
    return False


# ── Core homing state machine (shared by all options) ──────────────────────────

def ensure_homing_method(bus, node_id, name):
    """
    Verify 0x6098:00 Homing_Method == 35, writing it if it differs.

    Runs immediately before the homing FSM, for every mode. This one is NOT part of
    the power-cycle-gated pre-flight check: Homing_Method takes effect on the next
    homing command, so correcting it here is enough to carry straight on.

    Returns True if the drive reports 35 by the time we return.
    """
    tag = f"0x{HOMING_METHOD_INDEX:04X}:{HOMING_METHOD_SUB:02X} Homing_Method"
    current = decode_response(
        send_read_telegram(bus, node_id, HOMING_METHOD_INDEX, HOMING_METHOD_SUB),
        signed=True)   # i8

    if current is None:
        print(f"[{name}] {tag}: NO RESPONSE — cannot verify")
        return False
    if current == HOMING_METHOD_EXPECTED:
        print(f"[{name}] {tag} = {current}  PASS")
        return True

    print(f"[{name}] {tag} = {current}, expected {HOMING_METHOD_EXPECTED} — setting")
    send_write_telegram(bus, node_id, HOMING_METHOD_INDEX, HOMING_METHOD_SUB,
                        HOMING_METHOD_EXPECTED, 1)
    readback = decode_response(
        send_read_telegram(bus, node_id, HOMING_METHOD_INDEX, HOMING_METHOD_SUB),
        signed=True)
    if readback != HOMING_METHOD_EXPECTED:
        print(f"[{name}] {tag}: write failed (read back {readback})")
        return False
    print(f"[{name}] {tag} set to {HOMING_METHOD_EXPECTED}")
    return True


def run_home_state_machine(bus, node_id, name):
    """
    Zero the encoder at the CURRENT position and persist it.

      0. 0x6098 = 35     Homing_Method (verified/corrected first)
      1. 0x6060 = 6      Operation mode = Homing
      2. 0x6040 = 0x06   Controlword: ready (NOT enabled yet)
      3. 0x2690 = 10     Encoder data reset (zero here)
      4. 0x6040 = 0x0F   Controlword: enable operation
      5. 0x6040 = 0x1F   Controlword: execute homing
      6. 0x1010:01 = "save"  Persist to NVM
    """
    global home_value

    if not ensure_homing_method(bus, node_id, name):
        print(f"[{name}] Aborting homing — Homing_Method is not {HOMING_METHOD_EXPECTED}.")
        return

    print(f"[{name}] Homing state machine: mode 6 -> 0x06 -> enc reset -> 0x0F -> 0x1F -> save")
    send_write_telegram(bus, node_id, 0x6060, 0x00, MODE_HOMING, 1)   # Homing mode
    time.sleep(0.2)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x06, 2)          # ready, not enabled
    time.sleep(0.2)
    encoder_data_reset(bus, node_id)                                  # 0x2690 = 10
    time.sleep(0.5)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x0F, 2)          # enable
    time.sleep(0.2)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x1F, 2)          # execute homing
    time.sleep(0.5)

    # Persist: CANopen store = ASCII "save" (0x65766173) to 0x1010:01.
    print(f"[{name}] Saving parameters to NVM...")
    send_write_telegram(bus, node_id, 0x1010, 0x01, 0x65766173, 4)
    time.sleep(1.5)

    home_value = get_actual_position(bus, node_id)
    print(f"[{name}] Home set. Actual position now: {home_value}")


# ── Sweep / limit detection ──────────────────────────────────────────────────────

def wait_for_hard_limit(bus, node_id, name, timeout=300):
    """
    Drive is already commanded toward a hard stop. Wait until the drive signals a limit
    (following error / error code) AFTER it has genuinely started moving.

    The "started moving" guard is the second half of the startup-bug fix: a following-error
    seen before the axis is op-enabled and has actually moved is treated as stale, not a hit.

    Returns 'limit' (hit a stop), 'target_reached' (moved but no stop), or 'timeout'.
    """
    deadline = time.time() + timeout
    start_pos = get_actual_position(bus, node_id)
    moved = False
    was_op_enabled = False

    while not stop_event.is_set():
        err = get_error_code(bus, node_id)
        sw  = get_status_word(bus, node_id)
        pos = get_actual_position(bus, node_id)

        if sw is not None:
            if sw & SW_OP_ENABLED:
                was_op_enabled = True
            if (start_pos is not None and pos is not None
                    and abs(pos - start_pos) > JOG_STEP):
                moved = True

            follow = bool(sw & SW_FOLLOW_ERROR)
            print(f"  [{name}] SW=0x{sw & 0xFFFF:04X} err=0x{(err or 0):04X} "
                  f"pos={pos} moved={moved}")

            # Only accept a stop once the axis has actually been enabled AND moved.
            if was_op_enabled and moved and ((err not in (None, 0)) or follow):
                print(f"  [{name}] Hard limit hit (err=0x{(err or 0):04X} SW=0x{sw & 0xFFFF:04X})")
                return "limit"

            # Enabled + moved, then dropped out of enable without a stop -> reached target.
            if was_op_enabled and moved and not (sw & SW_OP_ENABLED):
                print(f"  [{name}] Reached target without a hard stop (SW=0x{sw & 0xFFFF:04X})")
                return "target_reached"

        if time.time() > deadline:
            print(f"  [{name}] Timeout after {timeout}s waiting for limit")
            return "timeout"
        time.sleep(0.3)
    return "timeout"


def sweep_to_limit(bus, node_id, name, target, label):
    """Drive to a hard stop, stabilize briefly, return the captured position."""
    print(f"[{name}] Sweeping to {label} limit...")
    move_to_target(bus, node_id, target)
    result = wait_for_hard_limit(bus, node_id, name)
    if result != "limit":
        print(f"[{name}] WARN: {label} sweep ended as '{result}', not a hard limit — "
              f"captured position may be unreliable")
    clear_fault(bus, node_id)
    time.sleep(LIMIT_STABILIZE_SEC)
    pos = get_actual_position(bus, node_id)
    print(f"[{name}] {label} limit position: {pos}")
    return pos


# ── Jog REPL (Option 2) ──────────────────────────────────────────────────────────

def jog_node(bus, node_id, name):
    """
    Interactive jog in Profile Position mode. Returns True if the operator finished
    aligning (run the state machine), False if they skipped the node.

    Commands: '+' = +JOG_STEP, '-' = -JOG_STEP, signed int = that many counts,
              'd' = done (align complete), 's' = skip node.
    """
    print(f"\n[{name}] Jog to align. Commands: + (+{JOG_STEP}), - (-{JOG_STEP}), "
          f"<signed int> = counts, d = done, s = skip")
    while True:
        pos = get_actual_position(bus, node_id)
        cmd = input(f"[{name}] pos={pos} > ").strip()
        if cmd == "d":
            return True
        if cmd == "s":
            print(f"[{name}] Skipped.")
            return False
        if cmd == "+":
            delta = JOG_STEP
        elif cmd == "-":
            delta = -JOG_STEP
        else:
            try:
                delta = int(cmd)
            except ValueError:
                print("  ? use +, -, a signed integer, d, or s")
                continue
        if pos is None:
            print("  cannot read position — retry")
            continue
        target = pos + delta
        print(f"  moving {delta:+d} -> target {target}")
        move_to_target(bus, node_id, target)
        time.sleep(0.5)


# ── Option orchestrators (single node, no threads) ───────────────────────────────

def option_full(bus, node_id, name):
    """Option 1: sweep both limits, go to midpoint, then home."""
    print(f"[{name}] Full homing on node {node_id}")
    if not ensure_clean_enabled(bus, node_id, name):
        return
    a = sweep_to_limit(bus, node_id, name, TAR_POS_CW, "CW")
    if stop_event.is_set():
        return
    if not ensure_clean_enabled(bus, node_id, name):
        return
    b = sweep_to_limit(bus, node_id, name, TAR_POS_CCW, "CCW")
    if stop_event.is_set():
        return
    if a is None or b is None:
        print(f"[{name}] ERROR: could not read both limits (a={a} b={b}) — aborting")
        return
    mid = (a + b) // 2
    print(f"[{name}] Midpoint (a+b)/2 = {mid}")
    move_to_target(bus, node_id, mid)

    # Wait until the midpoint is reached (no hard stop expected here).
    deadline = time.time() + MIDPOINT_TIMEOUT
    while not stop_event.is_set():
        pos = get_actual_position(bus, node_id)
        if pos is not None and abs(pos - mid) <= JOG_STEP:
            print(f"[{name}] Reached midpoint: {pos}")
            break
        if time.time() > deadline:
            print(f"[{name}] WARN: midpoint not reached within {MIDPOINT_TIMEOUT}s "
                  f"(pos={pos}, target={mid}) — aborting before homing")
            return
        time.sleep(0.5)
    if stop_event.is_set():
        return
    run_home_state_machine(bus, node_id, name)


def option_blind(bus, node_id, name):
    """Option 3: home at the current position."""
    print(f"[{name}] Blind homing on node {node_id}")
    if not ensure_clean_enabled(bus, node_id, name):
        return
    if stop_event.is_set():
        return
    run_home_state_machine(bus, node_id, name)


def option_align(bus, node_id, name):
    """Option 2: confirm alignment or jog, then home."""
    print(f"\n===== {name} (node {node_id}) =====")
    ans = input(f"Is {name} aligned? [y/n] ").strip().lower()
    if ans == "y":
        print(f"[{name}] Reported aligned — skipping.")
        return
    if not ensure_clean_enabled(bus, node_id, name):
        return
    if not jog_node(bus, node_id, name):
        return   # operator skipped
    run_home_state_machine(bus, node_id, name)


# ── Pre-flight OD check ──────────────────────────────────────────────────────────

def _sdo_response_size(response):
    """Infer the OD entry size in bytes from an expedited SDO upload response byte."""
    return {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(response.data[0], 4)


def _verify_and_fix_od(bus, node_id, index, subindex, expected):
    """
    Read one OD, compare to expected. If it differs, write the correct value and read back.
    Returns (status, changed): status is 'pass' | 'fixed' | 'fail'.
    """
    tag = f"[node {node_id}] 0x{index:04X}:{subindex:02X}"
    resp = send_read_telegram(bus, node_id, index, subindex)
    if resp is None:
        print(f"  {tag}  NO RESPONSE — cannot verify")
        return "fail", False
    current    = decode_response(resp)
    size_bytes = _sdo_response_size(resp)

    if current == expected:
        print(f"  {tag}  = {current}  PASS")
        return "pass", False

    print(f"  {tag}  = {current}, expected {expected} — FIXING")
    send_write_telegram(bus, node_id, index, subindex, expected, size_bytes)
    readback = decode_response(send_read_telegram(bus, node_id, index, subindex))
    if readback != expected:
        print(f"  {tag}  write failed (read back {readback}) — FAIL")
        return "fail", True
    print(f"  {tag}  corrected to {expected}")
    return "fixed", True


def preflight_od_check(bus, node_id):
    """
    Verify (and auto-fix + save) the pre-flight ODs on the TARGET NODE only:
      - 0x607E:00 (Invert_Dir)
      - 0x6410:13 (Invert_Dir_Motor)
      - 0x6099:03 (Homing_Power_On)   == 2   (steer nodes only)
      - 0x6099:05 (Home_Offset_Mode)  == 1   (steer nodes only)

    Returns True only if the node was already correct (homing may proceed). Returns False
    if it was unreachable/unwritable OR any value had to be corrected — a corrected node
    needs a power-cycle for the change to take effect, so we save to EEPROM and ask the
    operator to power-cycle and re-run.
    """
    print(f"===== Pre-flight: OD check (node {node_id}) =====")
    any_fail = False
    changed  = False

    if node_id in INVERT_DIR_EXPECTED:
        s1, c1 = _verify_and_fix_od(bus, node_id, INVERT_DIR_INDEX, INVERT_DIR_SUB,
                                    INVERT_DIR_EXPECTED[node_id])
        s2, c2 = _verify_and_fix_od(bus, node_id, INVERT_DIR_MOTOR_INDEX, INVERT_DIR_MOTOR_SUB,
                                    INVERT_DIR_MOTOR_EXPECTED[node_id])
        any_fail |= "fail" in (s1, s2)
        changed  |= c1 or c2
    else:
        print(f"  [node {node_id}] no direction-OD expectation on record — skipping 0x607E/0x6410")

    # Homing ODs are only maintained on the steer nodes.
    if node_id in STEER_NODES:
        s3, c3 = _verify_and_fix_od(bus, node_id, HOMING_6099_POWERON_INDEX,
                                    HOMING_6099_POWERON_SUB, HOMING_6099_POWERON_EXPECTED)
        s4, c4 = _verify_and_fix_od(bus, node_id, HOMING_6099_OFFMODE_INDEX,
                                    HOMING_6099_OFFMODE_SUB, HOMING_6099_OFFMODE_EXPECTED)
        any_fail |= "fail" in (s3, s4)
        changed  |= c3 or c4
    else:
        print(f"  [node {node_id}] not a steer node — skipping 0x6099:03 / 0x6099:05")

    if changed:
        print(f"  [node {node_id}] Saving corrected config to EEPROM...")
        send_write_telegram(bus, node_id, 0x1010, 0x01, 0x65766173, 4)
        time.sleep(1.5)
        print(f"  [node {node_id}] Saved.")

    print("===== OD check complete =====")
    if any_fail:
        print(f"Pre-flight OD check FAILED — node {node_id} could not be verified/written.")
        print("Check the adapter link, bus wiring/termination, --bitrate and node power.")
        print("Or pass --skip-dir-check, then re-run. Aborting.")
        return False
    if changed:
        print(f"Pre-flight ODs were corrected and saved on node {node_id}.")
        print("Power-cycle that driver so the changes take effect, then re-run homing.")
        return False
    print("All pre-flight ODs already correct — proceeding to homing.\n")
    return True


# ── Menu / entry point ───────────────────────────────────────────────────────────

MODE_DISPATCH = {
    "full":  option_full,
    "align": option_align,
    "blind": option_blind,
}

MODE_MENU = [
    ("1", "full",  "Full homing     (sweep both limits, go to midpoint, then home)"),
    ("2", "align", "Assisted align  (confirm/jog to alignment, then home)"),
    ("3", "blind", "Blind homing    (home at current position, no sweep/jog)"),
]


def prompt_mode():
    print("Select homing option:")
    for num, _mode, desc in MODE_MENU:
        print(f"  {num}) {desc}")
    choice = input("> ").strip()
    for num, mode, _desc in MODE_MENU:
        if choice in (num, mode):
            return mode
    print(f"Invalid choice: {choice!r}")
    return None


def prompt_node():
    known = ", ".join(f"{nid}={nm}" for nid, nm in sorted(NODE_NAMES.items()))
    print(f"Known nodes: {known}")
    raw = input("Node ID to home: ").strip()
    try:
        node_id = int(raw)
    except ValueError:
        print(f"Invalid node ID: {raw!r}")
        return None
    if not 1 <= node_id <= 127:
        print(f"Node ID out of CANopen range 1..127: {node_id}")
        return None
    return node_id


def main():
    parser = argparse.ArgumentParser(
        description="CANopen single-node auto-homing over a USB-CAN adapter (3 options).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--node", "-n", type=int,
                        help="CANopen node ID to home (1-127); prompts if omitted")
    parser.add_argument("--channel", "-c", "--interface", "-i", default=CAN_CHANNEL,
                        dest="channel",
                        help="adapter CAN port (0 = CAN1, 1 = CAN2), or 'can0' for socketcan")
    parser.add_argument("--bustype", "-b", default=CAN_BUSTYPE, help="python-can bus type")
    parser.add_argument("--bitrate", type=int, default=CAN_BITRATE,
                        help="bus bitrate in bit/s (ignored for socketcan)")
    parser.add_argument("--mode", "-M", choices=list(MODE_DISPATCH),
                        help="homing option (skips the menu)")
    parser.add_argument("--skip-dir-check", action="store_true",
                        help="skip the pre-flight OD check")
    parser.add_argument("--yes", "-y", action="store_true",
                        help="don't ask for confirmation on a non-steer node")
    args = parser.parse_args()

    mode = args.mode or prompt_mode()
    if mode is None:
        sys.exit(1)

    node_id = args.node if args.node is not None else prompt_node()
    if node_id is None:
        sys.exit(1)
    if not 1 <= node_id <= 127:
        print(f"Node ID out of CANopen range 1..127: {node_id}")
        sys.exit(1)
    name = NODE_NAMES.get(node_id, f"node{node_id}")

    # Homing objects are only maintained on the steer nodes (see SDO.md §5).
    if node_id not in STEER_NODES and not args.yes:
        print(f"WARNING: node {node_id} ({name}) is not a steer node {STEER_NODES}. "
              f"Homing is only commissioned on the steer motors, so the pre-flight check "
              f"below verifies less (0x6099:03/:05 are skipped).")
        if input("Continue anyway? [y/N] ").strip().lower() != "y":
            print("Aborted.")
            sys.exit(1)

    print(f"\nAdapter  : {args.bustype} channel {args.channel} @ {args.bitrate} bit/s")
    print(f"Mode     : {mode}")
    print(f"Node     : {node_id} ({name})\n")

    try:
        bus = open_bus(args.bustype, args.channel, args.bitrate, node_id)
    except Exception as exc:   # noqa: BLE001 — operator script, explain and exit
        print(f"ERROR: could not open the CAN adapter: {exc}")
        print("Checks: adapter plugged in and driver installed (dpinst64.exe from "
              "USB_CAN TOOL/driver); USB_CAN_Tool.exe closed; "
              "`pip install python-can canalystii libusb-package` done; correct --channel.")
        print('("No backend available" specifically means pyusb found no libusb — '
              'install libusb-package.)')
        sys.exit(1)

    with bus:
        try:
            # Fail fast and legibly if the node isn't there at all.
            if not probe_node(bus, node_id, name):
                sys.exit(1)

            # Pre-flight gate: correct config ODs must be in place (and take a power-cycle
            # to apply).
            if args.skip_dir_check:
                print("Skipping pre-flight OD check (--skip-dir-check).\n")
            elif not preflight_od_check(bus, node_id):
                sys.exit(1)

            MODE_DISPATCH[mode](bus, node_id, name)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt — stopping...")
            stop_event.set()
            emergency_halt(bus, node_id, name)
            time.sleep(1)
            print("Stopped.")
            sys.exit(0)
        except Exception as exc:   # noqa: BLE001 — operator script, report and halt
            print(f"[{name}] Error: {exc}")
            stop_event.set()
            emergency_halt(bus, node_id, name)
            sys.exit(1)

    print("\n===== Final Home Value =====")
    print(f"  {name} (node {node_id}): "
          f"{home_value if home_value is not None else 'N/A'}")


if __name__ == "__main__":
    main()
