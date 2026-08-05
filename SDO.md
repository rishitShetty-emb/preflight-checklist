# AMR CANopen OD & SDO Reference

Every CANopen object we touch on the AMR 6-DOF chassis, organized by **what it does**.
Each object appears once. Two subsystems talk to these drives:

Nodes: drive/travel (mode 3 Profile Velocity) = `wheel_f`=1, `wheel_rl`=3, `wheel_rr`=5;
steer (mode 1 Profile Position) = `steering_f`=2, `steering_rl`=4, `steering_rr`=6; master = 9.

## 1. Device control & FSA state

| Object | Sub | Name | Type | Value / notes | Used by |
|---|---|---|---|---|---|
| `0x6040` | `0x00` | Controlword | u16 | FSA walk `0x0000→0x0080→0x0006→0x0007→0x000F`; `0x103F` streaming (steer); homing `0x2F/0x3F`; halt `0x0006` | drv-w, home-w |
| `0x6041` | `0x00` | Statusword | u16 | FSA state / fault bit | drv-r, home-r, `live` |
| `0x6060` | `0x00` | Operation_Mode | i8 | 3 travel / 1 steer (driver); 6 Homing / 1 PP (homing tool) | drv-w, home-w |
| `0x603F` | `0x00` | Error Code (standard) | u16 | CiA-402 error code | home-r, `live` |
| `0x2601` | `0x00` | Error_State (vendor) | u16 | 16-bit fault bitfield; mapped to TPDO3 | `live` |
| `0x2602` | `0x00` | Error_State 2 | u16 | Extended errors (`0x2601` bit0 → here). Not currently read | — |

## 2. Motion feedback / telemetry (volatile — not to be verified)

| Object | Sub | Name | Type | Value / notes | Used by |
|---|---|---|---|---|---|
| `0x6063` | `0x00` | Pos_Actual (vendor, internal units) | i32 | via TPDO1 → RPDO staging `0x3000+` | drv-r, `live` |
| `0x606C` | `0x00` | Velocity_Actual | i32 | via TPDO1 → staging | drv-r, `live` |
| `0x6078` | `0x00` | Current actual (I_q, ‰ rated) | i16 | via TPDO2 | drv-r, `live` |
| `0x6077` | `0x00` | Torque actual | i16 | defined but **no TPDO4 on AMR** → unpopulated | — |
| `0x60F7` | `0x0B` | Drive/inverter temp (°C) | i16 | async throttled SDO — the only steady-state runtime bus SDO | drv-r, `live` |
| `0x6410` | `0x19` | Motor temp | i16 | referenced in code, **not in AMR manual OD list** → unverified/unpopulated | — |

## 3. Motion targets & profile limits (Not one time OD's config, subjected to change. no need to verify)

| Object | Sub | Name | Type | Value / notes | Used by |
|---|---|---|---|---|---|
| `0x607A` | `0x00` | Target_Position | i32 | RPDO1 (steer); homing setpoints | drv-w, home-w, `cmd` |
| `0x60FF` | `0x00` | Target_Velocity | i32 | via master TPDO staging; driver writes 0 on halt | drv-w, `cmd` |
| `0x6081` | `0x00` | Profile_Velocity | u32 | steer profile vel; homing speed | drv-w, home-w |
| `0x6083` | `0x00` | Profile_Acc | u32 | 107372 / 1638 | drv-w, `cfg` |
| `0x6084` | `0x00` | Profile_Dec | u32 | 107372 / 1638 | drv-w, `cfg` |
| `0x6085` | `0x00` | Quick_Stop_Dec | u32 | 654980 / 99942 | `cfg` |
| `0x607F` | `0x00` | Max_Speed (profile ceiling) | u32 | 89478484 / 13653333 | `cfg` |
| `0x6065` | `0x00` | Max_Following_Error | u32 | 524288 / 80000 | `cfg` |
| `0x6067` | `0x00` | Target_Pos_Window | u32 | 327 / 50 | `cfg` |
| `0x6068` | `0x00` | Pos_Window_Time | u16 | 10 | `cfg` |

## 4. Stop & safety behavior(Not one time OD's config, subjected to change. no need to verify)

| Object | Sub | Name | Type | Value | Used by |
|---|---|---|---|---|---|
| `0x605A` | `0x00` | Quick_Stop_Mode | i16 | cfg 0; driver writes option `2` (decel-on-ramp) at init | drv-w, `cfg` |
| `0x605B` | `0x00` | Shutdown_Stop_Mode | i16 | 0 | `cfg` |
| `0x605C` | `0x00` | Disable_Stop_Mode | i16 | 0 | `cfg` |
| `0x605D` | `0x00` | Halt_Mode | i16 | 1 (ramp stop) | `cfg` |
| `0x605E` | `0x00` | Fault_Stop_Mode | i16 | 0 | `cfg` |
| `0x6007` | `0x00` | Abort_Connection_Mode | i16 | cfg 0; driver toggles 0→3 (Quick-Stop on HB loss) at runtime | drv-w, `cfg` |
| `0x607D` | `0x01` | Soft_Positive_Limit | i32 | 0 (disabled) | `cfg` |
| `0x607D` | `0x02` | Soft_Negative_Limit | i32 | 0 (disabled) | `cfg` |
| `0x2010` | `0x19` | Limit_Function | u8 | 1 (limit-switch handling enabled) | `cfg` |

## 5. Direction & homing

| Object | Sub | Name | Type | Value | Used by |
|---|---|---|---|---|---|
| `0x607E` | `0x00` | Invert_Dir | u8 | 0 (**node 1 = 1**) | home-w, home-r, `cfg` |
| `0x6410` | `0x13` | Invert_Dir_Motor | u8 | 1 | home-w, home-r, `cfg` |
| `0x607C` | `0x00` | Home_Offset | i32 | 0 | `cfg` |
| `0x6098` | `0x00` | Homing_Method | i8 | 35 | `cfg` |
| `0x6099` | `0x01` | Homing_Speed_Switch | u32 | 5368708 / 819200 | `cfg` |
| `0x6099` | `0x02` | Homing_Speed_Zero | u32 | 1789568 / 273066 | `cfg` |
| `0x6099` | `0x03` | Homing_Power_On | u8 | **2** | home-w, home-r, `cfg` |
| `0x6099` | `0x04` | Homing_Current | i16 | 910 / 421 | `cfg` |
| `0x6099` | `0x05` | Home_Offset_Mode | u8 | 0 | `cfg` |
| `0x609A` | `0x00` | Homing_Accel | u32 | 53684 / 8192 | `cfg` |
| `0x2690` | `0x00` | Encoder Data Reset (vendor) | u8 | write 10 to reset multi-turn | home-w |

## 6. Control-loop tuning — DO NOT TOUCH

| Object | Sub | Name | Type | Value | Used by |
|---|---|---|---|---|---|
| `0x60F6` | `0x01` | Kcp (current-loop P) | u16 | 3600 / 2100 | `cfg` |
| `0x60F6` | `0x02` | Kci (current-loop I) | u16 | 56 / 28 | `cfg` |
| `0x60F6` | `0x03` | Speed_Limit_Factor | u16 | 10 | `cfg` |
| `0x60F9` | `0x01` | Kvp[0] (velocity P) | u16 | 5 / 70 | `cfg` |
| `0x60F9` | `0x02` | Kvi[0] (velocity I) | u16 | 0 | `cfg` |
| `0x60F9` | `0x07` | Kvi/32 (fine velocity I) | u16 | 10 / 0 | `cfg` |
| `0x60F9` | `0x03` | Notch_N | u8 | 45 | `cfg` |
| `0x60F9` | `0x04` | Notch_On | u8 | 0 | `cfg` |
| `0x60F9` | `0x05` | Speed_Fb_N | u8 | 7 | `cfg` |
| `0x60FB` | `0x01` | Kpp[0] (position gain) | i16 | 1000 | `cfg` |
| `0x60FB` | `0x02` | K_Velocity_FF | i16 | 256 | `cfg` |
| `0x60FB` | `0x03` | K_Acc_FF | i16 | 32767 | `cfg` |
| `0x60FB` | `0x05` | Pos_Filter_N | u16 | 1 | `cfg` |

## 7. Motor nameplate, current limit & brake

| Object | Sub | Name | Type | Value | Used by |
|---|---|---|---|---|---|
| `0x6410` | `0x01` | Motor_model | u16 | 12881 / 13366 | `cfg` |
| `0x6410` | `0x03` | Encoder resolution | u32 | 65536 / 10000 | `cfg` |
| `0x6073` | `0x00` | CMD_q_Max (max current, Arms) | u16 | 1028 / 1536 | `cfg` |
| `0x6410` | `0x11` | Brake duty cycle | u16 | 2250 | `cfg` |
| `0x6410` | `0x12` | Brake delay (ms) | u16 | 150 | `cfg` |

## 8. Communication & node identity

| Object | Sub | Name | Type | Value | Used by |
|---|---|---|---|---|---|
| `0x100B` | `0x00` | Node-ID | u8 | = node id (1..6) | `cfg` |
| `0x1005` | `0x00` | SYNC COB-ID | u32 | 128 (`0x80`) | `cfg` |
| `0x1006` | `0x00` | Comm cycle period (µs) | u32 | 4000 | `cfg` |
| `0x100C` | `0x00` | Guard_Time (ms) | u16 | 1000 | `cfg` |
| `0x100D` | `0x00` | Life_Time_Factor | u8 | 3 | `cfg` |
| `0x1016` | `0x01` | Heartbeat consumer | u32 | cfg `0x7F0100`; driver rewrites `0x0009012C` (node 9, 300 ms) at runtime | drv-w, `cfg` |
| `0x1017` | `0x00` | Heartbeat producer (ms) | u16 | 500 | `cfg` |
| `0x1010` | `0x01` | Store parameters → NVM | u32 | write ASCII `"save"` (`0x65766173`) | home-w |
| `0x1800`–`0x1803` | `0x01` | TPDO COB-ID | u32 | driver sets disable-bit to silence TPDOs during FSA | drv-w |
| `0x1400`–`0x1BFF` | — | RPDO/TPDO comm + mapping | — | **reprogrammed by driver at runtime** (RPDO1, TPDO1 pos+vel, TPDO2 sw+current, TPDO3 error) — do not verify live | drv-w |