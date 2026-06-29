# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## 1. Competition Overview (北邮 2026 电赛)

### 1.1 Task Summary

Design a **"Line-Tracing + Target-Operation Integrated Platform"** consisting of:
- A wheeled car chassis with automatic line-following capability
- A **gimbal or robotic-arm** actuator module that can aim at a target board and perform pointing/marking operations

The car must navigate a black-line track autonomously, stop at designated points, perform target tasks, and return — all without human intervention during a run.

### 1.2 Track / Course Layout

| Parameter | Value |
|-----------|-------|
| Track surface | White matte flat material |
| Minimum area | 220 cm × 120 cm |
| Track shape | Closed/unclosed: two straight segments + two symmetric semicircular arcs |
| Line width | 1.8 cm ± 0.2 cm (black line on white surface) |
| Key waypoints | **A** (start/finish), **B** (gimbal task stop), **C**, **D** |
| Target board position | 50 cm outside the AB segment, parallel to AB, height ≤ 50 cm |
| Target board spec | A4-size paper with bullseye center-dot + concentric scoring rings |

### 1.3 Required Tasks (with scoring)

| # | Task | Points | Time Limit |
|---|------|--------|------------|
| 1 | **自动巡迹** — Start at A, follow track 1 full lap, stop at A with buzzer+LED | 20 | ≤ 30 s |
| 2 | **定点瞄准** — Car placed anywhere on track; within 5 s, gimbal auto-aligns with bullseye and performs one pointing action | 20 | ≤ 5 s |
| 3 | **巡迹到靶位联动** — Start at A → follow to B → stop → gimbal aims → continue C → D → A; buzzer+LED at every waypoint | 20 | ≤ 40 s |
| 4 | **动态瞄准** *(bonus)* — While driving, gimbal continuously tracks the target; brief deviation in turns allowed | 15 | — |
| 5 | **同步绘制** *(bonus)* — While stationary at A, gimbal/arm draws a specified pattern on the target | 15 | — |
| 6 | **四圈连续运行** *(bonus)* — 4 continuous laps + one complete aiming task; shorter = better | 10 | minimize |
| 7 | Engineering quality | 10 | — |
| 8 | Design report (must describe AI tool usage in detail) | 10 | — |
| **Total** | | **120** | |

### 1.4 Hardware Rules & Restrictions

- **Chassis**: Wheeled only. **No treads, no Mecanum wheels.**
- **MCU**: **Must be TI chip** (current: MSPM0G3507 ✅)
- **Cameras**: Camera/vision system must **not be required** for basic driving or target task execution. The car and gimbal must complete path-following and basic pointing without vision.
  - Camera/K230 may only be used as auxiliary actuator-end positioning for bonus/extension tasks.
  - **Current decision**: K230 is NOT being purchased. All core tasks will be implemented without vision.
- **Dimensions**: Total size ≤ 25 cm × 15 cm × 25 cm
- **Power**: Gimbal/arm module must have independent power control
- **Modularity**: Chassis, gimbal, and HMI must support independent power-on and individual testing; mode/params configurable via buttons or UART

### 1.5 Evaluation & Submission

- Online video submission covering: overall appearance, all 6 task demonstrations
- AI tool usage must be described in the design report for bonus points (claudecode, opencode, codex encouraged)

---

## 2. Project Overview

BUPT 2026 Electronic Design Contest — line-following car using **TI MSPM0G3507** (Cortex-M0+ @ 80MHz). The car follows a black line using an 8-channel grayscale sensor array, with JY61P IMU for yaw angle, two hall encoders for wheel speed, and TB6612 dual H-bridge motor driver.

### Current Implementation Status

| Task | Status | Implementation |
|------|--------|----------------|
| 自动巡迹 (1 lap) | ⚠️ Legacy code exists, **pending hardware verification** | `track2()` / `track3()` |
| 四圈连续运行 | ⚠️ Legacy code exists, **pending hardware verification** | `track4()` |
| 定点瞄准 | ❌ Not implemented | Needs `gimbal.c/h` |
| 巡迹到靶位联动 | ❌ Not implemented | Needs `mission.c/h` + gimbal |
| 动态瞄准 | ❌ Not implemented | Needs gimbal + realtime yaw feed |
| 同步绘制 | ❌ Not implemented | Needs arm trajectory control |

---

## 3. Build & Flash

- **IDE:** Keil MDK (uVision) with ARMCLANG V6.23
- **Project file:** `user/project.uvprojx`
- **Device Pack:** TexasInstruments.MSPM0G1X0X_G3X0X_DFP.1.3.1
- **Build:** Open project in Keil → Build (F7). Target: `0 Error / 0 Warning`.
- **Flash:** Keil Debug/Download button (CMSIS-DAP via XDS110 or on-board debugger).

No CLI build tools are configured; all development happens inside Keil IDE.

---

## 4. Architecture

### 4.1 Execution Model

Three concurrent contexts sharing global variables:

1. **`main()` loop** (foreground, ~100Hz) — secondary key polling via `key_GetNum()` (`PA17`=mode cycle 1–4, `PA27`=set toggle), IMU data parsing from UART1 ring buffer (`imu_analysis()`), OLED display refresh (mode/set/yaw).
2. **`GROUP1_IRQHandler()`** (GPIO EXTI, in `encoder.c`) — **primary** button handler for PA17/PA27 with `delay_ms(10)` debounce; also handles all four encoder edge interrupts (PA12/PA13/PA24/PA15).
3. **`TIMG8_IRQHandler()`** (10ms ISR, background) — reads encoder deltas, then dispatches to the active `trackN()` function based on global `mode` and `set` variables. All PID computation and motor output happens here.

### 4.2 Layered Structure

```
user/              ← Application layer (your code)
  main.c           — system init, main loop, TIMG8 ISR dispatch
  xunji.c/h        — 8-ch grayscale sensor + track1/2/3/4 strategies
  _pid.c/h         — PID controller library (incremental + positional)
  encoder.c/h      — Quadrature decoder + button EXTI (GROUP1_IRQHandler)
  IMU.c/h          — JY61P IMU UART parser (0x55 0x53 angle frame)
  key.c/h          — Two-button input with debounce (secondary path)
  buzzer_light.c/h — Buzzer (PB8) + LED (PB26) indicator, auto-off via TIMG7
  track.c/h        — Track detection utility (get_line_color)
  isr.c            — All interrupt handlers (UART1 IMU, TIMG7 buzzer, stubs for unused)
  serical.c / seriacl.h — UART0 external sensor (0x75 frame protocol)
                    [note: header filename typo — seriacl.h not serical.h]
  ti_msp_dl_config.c/h — TI SysConfig-generated peripheral config

  [PLANNED - not yet created]
  gimbal.c/h       — 2-DOF servo gimbal control (PWM-based yaw+pitch)
  mission.c/h      — Full-lap state machine (IDLE→TRACE_TO_B→STOP_AT_B→GIMBAL_AIM→...)

ml_libs/           ← Hardware abstraction layer (do not modify)
  ml_gpio.c/h      — GPIO init/read/write wrapper
  ml_pwm.c/h       — PWM init/update wrapper
  ml_tim.c/h       — Timer interrupt init
  ml_uart.c/h      — UART init/send/receive
  ml_i2c.c/h       — Software I2C
  ml_oled.c/h      — OLED SSD1306 display driver
  ml_motor.c/h     — TB6612 motor Set_left_pwm() / Set_right_pwm()
  ml_delay.c/h     — delay_ms() / delay_us()
  ml_system.c/h    — system_init() (80MHz clock)
  ml_exti.c/h      — GPIO EXTI configuration
  ml_mpu6050.c/h   — MPU6050 driver (unused in current design, JY61P used instead)
  headfile.h       — Master include header (all SDK + ml_libs + user headers)

m0_sdk/            ← TI MSPM0 SDK (read-only, do not edit)
code/pid.h         ← Legacy PID header (not actively used; prefer user/_pid.h)
```

### 4.3 Critical Global Variables

| Variable | Defined in | Modified by | Read by |
|----------|-----------|-------------|---------|
| `mode` (1–4) | `encoder.c` | `GROUP1_IRQHandler` (PA17 interrupt, primary); `main.c` `key_GetNum()` (secondary) | TIMG8 ISR dispatch |
| `set` (0/1) | `encoder.c` | `GROUP1_IRQHandler` (PA27 interrupt, primary); `main.c` (secondary) | TIMG8 ISR dispatch |
| `yaw_angle_int` | `IMU.c` | `imu_analysis()` called from `main()` | `track3/4` yaw-based turns, `check()`, OLED display |
| `imu_flag` | `IMU.c` | `imu_uart_callback()` in UART1 ISR | `main()` loop trigger for `imu_analysis()` |
| `left_encoder` | `_pid.c` | TIMG8 ISR (from `read_encoder1()`) | `turn_pid()`, `speed_pid()` |
| `right_encoder` | `_pid.c` | TIMG8 ISR (from `read_encoder2()`) | `turn_pid()`, `speed_pid()` |
| `change_flag1` | `xunji.c` | track2/3/4 crossing detection | track2/3/4 state machine |
| `motorA_dir`, `motorB_dir` | `ml_motor.c` | `motor_target_set()` | PID sign logic |

### 4.4 Track Strategies (xunji.c)

The grayscale sensor reads D1–D8 as binary (1=white, 0=black). This polarity is because GPIOs are configured `IN_UP` (internal pull-up): on white surface the sensor output is high-impedance and the pull-up holds the pin HIGH; on black surface the sensor pulls the pin LOW. If you swap to a different sensor module with opposite polarity, all D==0/D==1 checks in the if/else chains must be inverted. Pattern matching uses if/else chains.

- **track1()** — *Calibration/test mode*: **stops** (PWM=0 + beep) when **any** sensor sees black (D=0); runs `turn_pid(10, 0)` to hold yaw=0 only when all sensors read white. Logic is **inverse** of normal line-following.

- **track2()** — *Single-lap, positive-direction*: Pure if/else PWM lookup table. `change_flag1` state machine:
  - `==0`: `turn_pid(10, 0)` — hold yaw=0 on straight
  - `==2`: `check(177); turn_pid(10, 177)` — positive-direction (CCW) sharp corner turn
  - `==4`: stop motors

- **track3()** — *Single-lap, negative-direction*: Same PWM lookup table, but uses **negative** angle targets:
  - `==0`: `turn_pid(10, -42)` — hold negative yaw on straight
  - `==2`: `check(-137); turn_pid(10, -137)` — reverse-direction (CW) sharp corner turn
  - `==4`: stop motors
  - For ambiguous single-sensor patterns (e.g. `01111111`, `11111110`), `yaw_angle_int` range checks (`-120<yaw<-90`, `-60<yaw<-20`) determine turn direction

- **track4()** — *4-lap continuous, negative-direction*: Extended state machine, `change_flag1` = 0→2→4→6→8→10→12→14→16(stop). Each even crossing alternates straight-hold vs corner-turn with slightly varying targets per lap:
  - Lap 1: `turn_pid(10,-42)` → `check(-138); turn_pid(10,-138)`
  - Lap 2: `turn_pid(10,-46)` → `check(-141); turn_pid(10,-141)`
  - Lap 3: `turn_pid(10,-47)` → `check(-142); turn_pid(10,-142)`
  - Lap 4: `turn_pid(10,-50)` → `check(-142); turn_pid(10,-142)` → stop at `==16`

### 4.5 PID Architecture (_pid.c)

Two cascaded loops in `turn_pid(base, target)`:
- **Outer loop** (angle, positional PID): `yaw_angle_int` → `angle.target` → outputs `angle.out` as a speed offset
- **Inner loop** (speed, incremental PID): motorA target = `base - angle.out`, motorB target = `base + angle.out` → PWM output via `Set_left_pwm()` / `Set_right_pwm()`

PID parameters (set in `main.c`):
```c
pid_init(&angle,  POSITION_PID, 0.04117, 0,  0.102);  // angle outer loop
pid_init(&motorA, DELTA_PID,    10,      10, 5);       // left wheel speed
pid_init(&motorB, DELTA_PID,    10,      10, 5);       // right wheel speed
```

`check(target)` handles yaw angle wrap-around at ±180° before `turn_pid()` to prevent direction reversal.

### 4.6 Key Pin Assignments

> **Note**: `ml_motor.h` header comments are **outdated** and do not match `ml_motor.c`. Use this table as the authoritative reference.

| Function | Pins |
|----------|------|
| Left motor PWM | TIMG6 CC0; direction: PB24 (AIN1) / PB20 (AIN2) |
| Right motor PWM | TIMG6 CC1; direction: PB19 (BIN1) / PB18 (BIN2) |
| TB6612 STBY | PB16 (always HIGH to enable) |
| Left encoder A/B | PA12 (RISING EXTI) / PA13 (FALLING EXTI) |
| Right encoder A/B | PA24 (RISING EXTI) / PA15 (FALLING EXTI) |
| Grayscale D1–D8 | PB13 / PB15 / PA31 / PA28 / PB1 / PB4 / PB17 / PB12 |
| IMU (JY61P) UART1 | PA8 (TX) / PA9 (RX), 9600 baud |
| OLED I2C | PB2 (SCL) / PB3 (SDA) |
| Buzzer | PB8 (active HIGH, auto-off via TIMG7) |
| LED | PB26 (active HIGH, auto-off via TIMG7) |
| Button 1 (mode) | PA17 (EXTI + polling) |
| Button 2 (set) | PA27 (EXTI + polling) |
| Gimbal yaw servo (planned) | **TBD** — ⛔ DO NOT use PB6/PB7 (already occupied by motor PWM, see below) |
| Gimbal pitch servo (planned) | **TBD** — ⛔ DO NOT use PB6/PB7 (already occupied by motor PWM, see below) |

---

## 5. Development Roadmap (新题目适配)

To adapt the existing codebase for the 2026 competition, the following work is needed:

### Priority 1 — Task 1: 自动巡迹 (20pts, already 80% done)
- [ ] Verify `track2()` completes 1 lap within 30 s on the actual 2026 track layout
- [ ] Confirm A→A stop with buzzer+LED (currently `change_flag1==4` stops — check if waypoint B detection is needed)
- [ ] Tune PWM values for 2026 track dimensions

### Priority 2 — Task 3: 巡迹到靶位联动 (20pts)
Create `mission.c/h` with state machine:
```c
typedef enum {
    TASK_IDLE = 0,
    TASK_TRACE_TO_B,   // follow line until change_flag1==2 (crossing at B)
    TASK_STOP_AT_B,    // stop + beep + trigger gimbal
    TASK_GIMBAL_AIM,   // wait for gimbal to complete aiming (≤5s)
    TASK_TRACE_TO_C,   // resume tracking
    TASK_TRACE_TO_D,
    TASK_TRACE_TO_A,   // final lap back to start
    TASK_FINISH        // stop + beep + LED
} task_state_t;
```
- `change_flag1` from `xunji.c` drives the state transitions
- Add `mode==5` for this mission mode in TIMG8 ISR and `main.c`

### Priority 3 — Task 2: 定点瞄准 (20pts)
Create `gimbal.c/h`:
```c
void gimbal_init(void);                      // init PWM for TBD pins (NOT PB6/PB7 — those are motor PWM)
void gimbal_set_angle(int yaw, int pitch);   // set servo angles via PWM duty
void gimbal_aim_B(void);                     // fixed angles calibrated to target at B
```
- **⚠️ Servo PWM timing**: 50Hz (20ms period). Safe test range: **1300–1700μs**. Full supported range: **500–2500μs**.
  - **Do NOT sweep the full 500–2500μs range before testing mechanical limits** — may hit hard stops.
  - Gimbal has two servos: one 180° range, one 270° range. Calibrate each separately.
- **⚠️ Pin selection required before writing gimbal.c**: Run the pin audit task below (see PINMAP.md) to identify free Timer+PWM pin combinations. Candidate timers: TIMG0 (PA12/PA13), TIMG7 (PA17/PA18), TIMG12 (PA14/PA24) — **all subject to conflict check**.
- Calibrate `gimbal_aim_B()` fixed angles on actual hardware (no vision needed)
- Add `mode==6` for standalone gimbal test

### Priority 4 — Task 6: 四圈连续运行 (10pts, already implemented as track4)
- [ ] Add gimbal aim trigger at B crossing in `track4()` (when `change_flag1==2`)
- [ ] Verify total time ≤ competition limit

### Priority 5 — Bonus Tasks
- **动态瞄准**: Continuously update gimbal angles based on `yaw_angle_int` during driving
- **同步绘制**: Implement arm trajectory at A position (requires mechanical arm, not just gimbal)

---

## 6. Adding a New Track Mode

1. Write `track5()` in `xunji.c` and declare in `xunji.h`.
2. In `GROUP1_IRQHandler()` (`encoder.c`): change wrap-around `if(mode==5)` → `mode=1` to allow mode 5.
3. In `TIMG8_IRQHandler()` (`main.c`): add `if(mode==5 && set==1) track5();`.
4. In `main()` key polling section: update `if(mode==5)` wrap.
5. Tune PWM values and angle thresholds per the physical track.

---

## 7. Key Gotchas & Notes

- **`change_flag1` is never reset between `set` toggles.** If a run is stopped mid-lap and restarted, `change_flag1` will be at a wrong value. Add a reset in the mode/set change logic if needed.
- **`check(target)` must be called before `turn_pid(target)` for large angle turns** (>90°) to prevent the yaw from wrapping in the wrong direction.
- **Dual button path conflict**: PA17/PA27 are handled by both `GROUP1_IRQHandler` (EXTI) and `key_GetNum()` (polling in main). The interrupt fires first; if both execute, `mode` increments twice. Verify in hardware whether the EXTI is actually triggering on these pins (check `ti_msp_dl_config.c` for EXTI enable).
- **`seriacl.h` filename typo**: The header file is `seriacl.h` (not `serical.h`). Include accordingly.
- **`ml_motor.h` pin comments are wrong**: Ignore the comment block listing `PWMA->2.5` etc. — those are old, incorrect. Use the pin table in §4.6 above.
- **TIMG7 auto-off for buzzer/LED**: `beep()` / `light_on()` in `buzzer_light.c` start TIMG7; `TIMG7_IRQHandler()` in `isr.c` turns them off after timeout. Do not call `DL_TimerG_disablePower(TIMG7)` manually outside the ISR.
- **⛔ PB6/PB7 are NOT available for gimbal**: `ml_pwm.c` hardcodes TIMG6 CC0→PB6 and TIMG6 CC1→PB7. Since `motor_init()` calls `pwm_init(TIMG6, CC0/CC1, 1000)`, these pins are taken by the motor driver. Any gimbal PWM must use a different TIMG instance. Run the pin audit (see `docs/PINMAP.md`) to select safe pins before writing `gimbal.c`.
- **`UART0_IRQHandler` is defined in two files**: The active handler lives in `serical.c` (lines 54–65); `isr.c` (lines 47–55) has a **commented-out** duplicate. If you uncomment the `isr.c` version without removing or commenting out the one in `serical.c`, the linker will produce a duplicate symbol error. Always keep only one definition active.
