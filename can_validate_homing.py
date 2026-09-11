#!/usr/bin/env python3
"""
CANopen Steer-Motor Homing Validation Script
---------------------------------------------
Run AFTER can_home_motors.py has completed homing on the three AMR steer
motors (Front=node 2, Left=node 4, Right=node 6). Confirms that each
node's absolute multi-turn position -- the thing homing just established --
actually survives a real power cycle of the drivers, i.e. that homing
"stuck" and the robot will still be correctly zeroed on the next boot.

Sequence:
  1. For each node: move by a fixed relative displacement (MOVE_COUNTS)
     away from its current (home) position, driven/settled via the
     standard 0x6064 Position_Actual_Value / 0x607A target-position pair
     (same registers can_home_motors.py uses for moves). This matters
     because checking a drive that's still sitting at/near zero would not
     prove the multi-turn count survives a power cycle -- moving off zero
     first means the comparison actually exercises the encoder's
     multi-turn tracking.
  2. Once settled, read the vendor multi-turn object 0x3000:17 on each
     node and record it as the "before" value. This is deliberately a
     different register from the one used to drive the move (see the
     note below).
  3. Prompt the operator to cut power to the drivers.
  4. Prompt the operator to release the E-stop / restore power, then wait
     for each node to reappear on the bus (polled via 0x3000:17).
  5. Re-read 0x3000:17 on each node as the "after" value.
  6. Compare |after - before| against TOLERANCE_COUNTS (default 5, per
     spec). Nodes within tolerance are reported as homing VERIFIED; nodes
     outside tolerance, or that never came back on the bus, are reported
     as homing NOT VERIFIED.

Multi-turn value note: 0x3000:17 (vendor-specific) is the value actually
being validated; 0x6064/0x607A (standard CiA-402 objects) are only used
to drive and confirm the validation move itself. These are kept as two
separate telemetry paths — get_actual_position() for movement,
get_multiturn_value() for the before/after comparison — since 0x3000:17
is not guaranteed to share units/scaling with the profile-position
registers, and using it as a move target could send an axis to a
nonsensical position. If this ever needs to change, edit
MULTITURN_INDEX / MULTITURN_SUB below.

This script does not modify homing itself and does not write to NVM; it
is read/move/compare only (aside from the moves needed to displace each
axis before the power-cycle test).
"""
import argparse
import subprocess
import sys
import time
import can

# === CONFIGURATION ===
MOTORS = {
    "Front": 2,
    "Left":  4,
    "Right": 6,
}
CAN_INTERFACE = "can0"
CAN_BUSTYPE   = "socketcan"

PROFILE_SPEED     = 200
PROFILE_SPEED_ENC = 17500 * PROFILE_SPEED   # encoder units per RPM, same scale as homing script

# How far (encoder counts) to move each wheel off its home position before
# cutting power. Signed: positive = one direction, negative = the other.
# Override with --move-counts if 100000 counts isn't a sensible "certain
# extent" of travel for your gearing/encoder resolution.
MOVE_COUNTS = 300000

# Allowed drift, in encoder counts, between the pre- and post-power-cycle
# reading of 0x6064 before a node is flagged as homing NOT VERIFIED.
TOLERANCE_COUNTS = 5

MOVE_SETTLE_COUNTS   = 50    # "close enough" to the validation-move target to call it arrived
MOVE_TIMEOUT_SEC     = 60    # give up waiting for the validation move to finish
RECONNECT_TIMEOUT_SEC = 180  # how long to wait for a node to reappear after power is restored
RECONNECT_POLL_SEC    = 1.0

MULTITURN_INDEX = 0x3000   # vendor-specific -- multi-turn absolute position, counts
MULTITURN_SUB   = 0x17

# One-time GPIO pre-flight -- must run BEFORE anything else (same call as the
# gpio_preflight() step in the latest can-home-motors). Equivalent to:
#   sudo gpioset gpiochip2 2=0 3=0 6=1
GPIO_PREFLIGHT_CMD = ["sudo", "gpioset", "gpiochip2", "2=0", "3=0", "6=1"]

# CiA-402 operation modes
MODE_PROFILE_POSITION = 1

# Statusword bits
SW_OP_ENABLED = 0x0004   # bit 2

# CANopen SDO COB-IDs (same scheme as can_home_motors.py)
SDO_TX_BASE = 0x600
SDO_RX_BASE = 0x580
SDO_READ_RESPONSES = (0x43, 0x4B, 0x4F)


# ── SDO helpers (identical to can_home_motors.py) ───────────────────────────
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
        try:
            bus.send(msg)
        except can.CanError:
            time.sleep(0.02)
            continue
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


# ── Drive telemetry ──────────────────────────────────────────────────────────
def get_actual_position(bus, node_id):
    """0x6064 Position_Actual_Value -- used to drive/settle the validation move.
    This is the register move_to_target()/0x607A already agree on (same as
    can_home_motors.py), so movement control stays independent of whatever
    units/scaling the vendor multi-turn object below uses."""
    return decode_response(send_read_telegram(bus, node_id, 0x6064, 0x00), signed=True)


def get_multiturn_value(bus, node_id):
    """0x3000:17 -- the vendor multi-turn value actually being validated across
    the power cycle. Read independently of movement; never used as a move
    target."""
    return decode_response(
        send_read_telegram(bus, node_id, MULTITURN_INDEX, MULTITURN_SUB), signed=True)


def get_error_code(bus, node_id):
    return decode_response(send_read_telegram(bus, node_id, 0x603F, 0x00))


def get_status_word(bus, node_id):
    return decode_response(send_read_telegram(bus, node_id, 0x6041, 0x00))


# ── Drive control primitives (same as can_home_motors.py) ─────────────────────
def clear_fault(bus, node_id):
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
    send_write_telegram(bus, node_id, 0x6060, 0x00, MODE_PROFILE_POSITION, 1)
    send_write_telegram(bus, node_id, 0x6081, 0x00, PROFILE_SPEED_ENC, 4)
    send_write_telegram(bus, node_id, 0x607A, 0x00, pos, 4)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x2F, 2)
    send_write_telegram(bus, node_id, 0x6040, 0x00, 0x3F, 2)


def ensure_clean_enabled(bus, node_id, name, timeout=10):
    """Reboot, clear any stale fault, confirm err==0 and no fault bit, then enable."""
    print(f"[{name}] Ensuring clean enabled state...")
    reboot_driver(bus, node_id)
    clear_fault(bus, node_id)
    deadline = time.time() + timeout
    while True:
        err = get_error_code(bus, node_id)
        sw  = get_status_word(bus, node_id)
        if err is not None and sw is not None:
            faulted = bool(sw & 0x0008)
            if err == 0 and not faulted:
                break
            clear_fault(bus, node_id)
        if time.time() > deadline:
            print(f"[{name}] WARN: could not reach a clean state "
                  f"(err={err} sw=0x{(sw or 0) & 0xFFFF:04X}) — aborting node")
            return False
        time.sleep(0.3)
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
    err = get_error_code(bus, node_id)
    print(f"[{name}] WARN: not Operation-Enabled after enable "
          f"(SW=0x{(sw or 0) & 0xFFFF:04X} err=0x{(err or 0):04X})")
    return False


# ── One-time GPIO pre-flight ────────────────────────────────────────────────
def gpio_preflight():
    """
    Drive the required GPIO lines once, before anything else -- identical to
    the gpio_preflight() step in the latest can-home-motors, so this script
    puts the controller-compute board into the same state (recovery active,
    halt disabled, dev mode on) before it opens the CAN bus. If it fails, tell
    the operator and abort -- the CAN controllers must be brought down first.
    """
    print("===== GPIO pre-flight: " + " ".join(GPIO_PREFLIGHT_CMD) + " =====")
    try:
        subprocess.run(GPIO_PREFLIGHT_CMD, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError, OSError) as exc:
        print(f"GPIO pre-flight command FAILED: {exc}")
        print("kill the controllers using supervisord")
        sys.exit(1)
    print("GPIO pre-flight lines set.\n")


# ── Step 1+2: displace each wheel and record the "before" position ──────────
def move_relative_and_record(bus, node_id, name, delta):
    """
    Move `delta` encoder counts off the current position (driven/settled via
    0x6064, same as can_home_motors.py), then, once settled, read and return
    0x3000:17 -- the vendor multi-turn value being validated. Returns None on
    any failure (unreachable node, error during the move, or timeout).
    """
    start = get_actual_position(bus, node_id)
    if start is None:
        print(f"[{name}] ERROR: could not read starting position — skipping node")
        return None

    target = start + delta
    print(f"[{name}] Validation move: {start} -> {target} ({delta:+d} counts)")
    move_to_target(bus, node_id, target)

    deadline = time.time() + MOVE_TIMEOUT_SEC
    settled = False
    while time.time() < deadline:
        err = get_error_code(bus, node_id)
        pos = get_actual_position(bus, node_id)
        if err not in (None, 0):
            print(f"[{name}] ERROR during validation move: err=0x{err:04X} pos={pos}")
            return None
        if pos is not None and abs(pos - target) <= MOVE_SETTLE_COUNTS:
            print(f"[{name}] Move complete. Position: {pos}")
            settled = True
            break
        time.sleep(0.3)

    if not settled:
        print(f"[{name}] WARN: validation move timed out — recorded value may be unreliable")

    value = get_multiturn_value(bus, node_id)
    print(f"[{name}] Recorded 0x{MULTITURN_INDEX:04X}:{MULTITURN_SUB:02X} = {value}")
    return value


# ── Step 4-5: wait for a node to come back after the power cycle ────────────
def wait_for_reconnect(bus, node_id, name, timeout=RECONNECT_TIMEOUT_SEC):
    """Poll 0x3000:17 until the node answers again, or time out."""
    print(f"[{name}] Waiting for node {node_id} to reappear on the bus...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        value = get_multiturn_value(bus, node_id)
        if value is not None:
            print(f"[{name}] Back online. 0x{MULTITURN_INDEX:04X}:{MULTITURN_SUB:02X} = {value}")
            return value
        time.sleep(RECONNECT_POLL_SEC)
    print(f"[{name}] ERROR: node {node_id} did not respond within {timeout}s of power restore")
    return None


# ── Report ────────────────────────────────────────────────────────────────────
def print_report(before, after, tolerance):
    """
    Print the pre-/post-power-cycle 0x3000:17 multi-turn value for every node,
    the delta between them, and the verdict against `tolerance`, then a final
    successful/not-successful summary.
    """
    reg_tag = f"0x{MULTITURN_INDEX:04X}:{MULTITURN_SUB:02X}"
    print("\n===== Homing Validation Report =====")
    print(f"Multi-turn value register: {reg_tag}   Tolerance: +/-{tolerance} counts\n")
    print(f"  {'Node':6s}  {'Before':>12s}  {'After':>12s}  {'Delta':>8s}  Result")
    print(f"  {'-'*6}  {'-'*12}  {'-'*12}  {'-'*8}  {'-'*12}")

    verified, not_verified = [], []
    for name in before:
        b = before.get(name)
        a = after.get(name)
        if b is None or a is None:
            print(f"  {name:6s}  {str(b):>12s}  {str(a):>12s}  {'--':>8s}  NOT VERIFIED (missing reading)")
            not_verified.append(name)
            continue
        delta = a - b
        ok = abs(delta) <= tolerance
        status = "VERIFIED" if ok else "NOT VERIFIED"
        print(f"  {name:6s}  {b:>12d}  {a:>12d}  {delta:>+8d}  {status}")
        (verified if ok else not_verified).append(name)

    print("\nHoming successful for : " + (", ".join(verified) if verified else "none"))
    print("Homing NOT successful for : " + (", ".join(not_verified) if not_verified else "none"))
    return verified, not_verified


# ── Main flow ─────────────────────────────────────────────────────────────────
def run_validation(motors, interface, bustype, move_counts, tolerance):
    before = {}
    with can.interface.Bus(interface=bustype, channel=interface) as bus:
        print("===== Step 1/2: Displace each wheel and record pre-power-cycle position =====")
        for name, node_id in motors.items():
            if not ensure_clean_enabled(bus, node_id, name):
                before[name] = None
                continue
            pos = move_relative_and_record(bus, node_id, name, move_counts)
            before[name] = pos

    if all(v is None for v in before.values()):
        print("\nNo node could be moved/read — aborting before asking for a power cycle.")
        return {}, list(motors)

    print("\n===== Step 3: Cut power =====")
    input("Cut power to the drivers now, then press Enter to continue... ")

    print("\n===== Step 4: Restore power =====")
    input("Release the E-stop / restore power to the drivers, then press Enter... ")

    after = {}
    with can.interface.Bus(interface=bustype, channel=interface) as bus:
        print("\n===== Step 5: Re-read post-power-cycle position =====")
        for name, node_id in motors.items():
            if before.get(name) is None:
                after[name] = None
                continue
            after[name] = wait_for_reconnect(bus, node_id, name)

    print("\n===== Step 6: Compare =====")
    return print_report(before, after, tolerance)


def prompt_nodes():
    names = input(f"Nodes [all] ({' '.join(MOTORS)}): ").strip()
    if not names:
        return dict(MOTORS)
    chosen = {}
    for token in names.split():
        if token in MOTORS:
            chosen[token] = MOTORS[token]
        else:
            print(f"  ignoring unknown motor: {token!r}")
    return chosen


def main():
    parser = argparse.ArgumentParser(
        description="Validate that homing on the CANopen steer motors survives a power cycle.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--interface", "-i", default=CAN_INTERFACE, help="CAN interface name")
    parser.add_argument("--bustype", "-b", default=CAN_BUSTYPE, help="python-can bus type")
    parser.add_argument("--motors", "-m", nargs="*",
                         help="subset of motors to validate (e.g. Front Left); default all")
    parser.add_argument("--move-counts", type=int, default=MOVE_COUNTS,
                         help="relative encoder-count displacement applied before the "
                              "power-cycle test (signed; negative = opposite direction)")
    parser.add_argument("--tolerance", type=int, default=TOLERANCE_COUNTS,
                         help="allowed +/- drift (encoder counts) after the power cycle")
    args = parser.parse_args()

    # One-time GPIO pre-flight -- must succeed before anything else runs.
    gpio_preflight()

    if args.motors is not None:
        motors = {n: MOTORS[n] for n in args.motors if n in MOTORS}
        if not motors:
            print(f"No matching motors. Available: {list(MOTORS)}")
            sys.exit(1)
    else:
        motors = prompt_nodes() if sys.stdin.isatty() else dict(MOTORS)

    print(f"\nInterface   : {args.interface} ({args.bustype})")
    print(f"Motors      : {list(motors)}")
    print(f"Move counts : {args.move_counts:+d}")
    print(f"Tolerance   : +/-{args.tolerance} counts\n")

    try:
        verified, not_verified = run_validation(
            motors, args.interface, args.bustype, args.move_counts, args.tolerance)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt — stopping...")
        sys.exit(0)

    sys.exit(0 if not not_verified else 1)


if __name__ == "__main__":
    main()
