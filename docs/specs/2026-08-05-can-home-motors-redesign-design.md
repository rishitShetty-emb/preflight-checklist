# can-home-motors redesign — design

Date: 2026-08-05
Repo: preflight-checklist
Target file: `can-home-motors` (single operator script; refactored, not split)

## Problem

The current `can-home-motors` only does one thing (full sweep + midpoint home of the
three steer motors) and has three issues:

1. **Missing state-machine step.** The encoder reset is done as a separate rebooted step
   instead of inside the mode-6 controlword bracket. The correct homing sequence keeps the
   controlword at `0x06` in Homing mode, resets the encoder, then advances `0x0F → 0x1F`.
2. **Startup bug.** If the drives were already faulted when the script starts, the first
   sweep's `wait_for_following_error` reads the *stale* fault/following-error bit and returns
   "limit hit" without the axis moving — capturing a bogus position and producing a partial
   home. The operator must run the script twice.
3. **No alternatives.** Midpoint `(a+b)/2` is not always accurate and can drift; there is no
   assisted-alignment path and no blind-home path.

Also: preflight OD checks miss `0x6099:05` (Home_Offset_Mode), which must be `1` on steer motors.

## Scope

- Steer motors only: Front=node 2, Left=node 4, Right=node 6. (Travel nodes 1/3/5 are not homed.)
- Raw-socketcan SDO, standalone, run before the ros2_canopen stack starts.
- No 120 s settle waits anymore — fine alignment/drift is handled by Option 2.

## Core homing state machine (shared by all options)

`run_home_state_machine(bus, node)`:

| Step | Object | Value | Meaning |
|---|---|---|---|
| 1 | `0x6060:00` | `6` | Operation mode = Homing |
| 2 | `0x6040:00` | `0x06` | Controlword: ready (NOT enabled) |
| 3 | `0x2690:00` | `10` | Encoder data reset (zero multi-turn here) |
| 4 | `0x6040:00` | `0x0F` | Controlword: enable operation |
| 5 | `0x6040:00` | `0x1F` | Controlword: execute homing |
| 6 | `0x1010:01` | `"save"` (`0x65766173`) | Persist to NVM |

The NVM save is part of the core sequence (runs every time a node is homed).

## Startup bug fix

New helper `ensure_clean_enabled(bus, node)`, called before any sweep or jog:

1. Fault-reset edge: controlword `0x80` → `0x06`.
2. Poll `0x603F` (error code) until `0` AND statusword (`0x6041`) shows not-faulted
   (bit 3 clear), up to a timeout.
3. Enable the drive.

`wait_for_following_error` additionally requires the drive to be **op-enabled and actually
moving** (observed position change / left the start) before it will accept a following-error
as a real limit hit — so a stale fault can never be misread as "at the stop."

## Options

Selected via interactive menu (default when run with no args); `--mode {full,align,blind}`
and `--motors` flags bypass the prompts for scripting.

```
$ ./can-home-motors
Select homing option:
  1) Full homing     (sweep both limits, go to midpoint, then home)
  2) Assisted align  (per node: confirm/jog to alignment, then home)
  3) Blind homing    (home at current position, no sweep/jog)
> 1
Nodes [all]: Front Left Right
```

### Option 1 — Full homing (parallel across nodes)

Per node, one thread each:
`ensure_clean_enabled` → sweep CW into hard stop → ~1 s stabilize → read `a`
→ sweep CCW into hard stop → ~1 s → read `b` → move to `(a+b)//2` → `run_home_state_machine`.

### Option 2 — Assisted alignment (sequential, node-by-node)

For each selected node:
1. Prompt: `Is <node> aligned? [y/n]`.
2. `y` → skip to next node.
3. `n` → jog REPL until aligned:
   - `+` → +10000 counts, `-` → −10000 counts
   - a signed integer → that many counts (e.g. `2500`, `-40000`)
   - `d` → done (run state machine), `s` → skip node
   - Jog mechanism: **Profile Position mode (mode 1)**, absolute move to
     `(current 0x6064) + delta`, via the existing `move_to_target` path
     (`0x607A` target, `0x6081` speed, controlword `0x2F → 0x3F`). Re-reads actual position
     each step.
4. On `d` → `run_home_state_machine(node)`.

### Option 3 — Blind homing (parallel across nodes)

Per node: `ensure_clean_enabled` → `run_home_state_machine`. No sweep, no jog. Assumes the
axis is already in position.

## Preflight OD gate

`preflight_od_check` extended to verify, on the steer nodes, with the existing
auto-fix → save → require-power-cycle-and-rerun semantics:

| Object | Sub | Expected | Note |
|---|---|---|---|
| `0x607E` | `0x00` | per-node (node 1 = 1, else 0) | Invert_Dir |
| `0x6410` | `0x13` | 1 | Invert_Dir_Motor |
| `0x6099` | `0x03` | 2 | Homing_Power_On |
| `0x6099` | `0x05` | 1 | Home_Offset_Mode (NEW) |

Runs for all options unless `--skip-dir-check`.

## Decisions / open assumptions

- **Jog = absolute-increment** in Profile Position mode (read pos, add delta, reuse
  `move_to_target`), not CiA-402 relative mode — reuses the tested path; net motion identical.
- **Jog re-reads actual position** each step (increments track real position, natural for a
  bench jog) rather than accumulating a commanded setpoint.
- **`0x6099:05` preflight** auto-fixes and forces a power-cycle like the other direction ODs.
  If Home_Offset_Mode takes effect without a power-cycle, change to fix-in-place.
- No 120 s settle; ~1 s stabilization before reading a swept limit position.

## Out of scope

- Travel-motor homing.
- Changing the `.cdi` files (the verifier's `verify-motor-config` override handling is separate).
