# Project Context: Closed-Loop Encoder Control for CHMT

This document provides full context for the Smoothieware-CHMT firmware project.
Read this alongside `CLAUDE.md` (which contains the core working principles).

## What This Project Is

The CHMT pick-and-place machine uses stepper motors for X and Y axes. Smoothieware
is an open-loop stepper firmware — it trusts that every commanded step physically
happens. At high speeds, stepper motors skip steps (magnetic flux alternates faster
than the motor can follow). The standard workaround is to run slowly enough that
skips are statistically impossible.

We are adding **closed-loop control** using hardware encoders on X and Y. The encoder
tells us the actual physical position regardless of whether steps were skipped.
This lets us run faster because we no longer need a safety margin against skipped steps.

## Architecture Overview

### Two systems, one motion plan

**OpenPnP** (host PC) does all motion planning using its `Simulated3rdOrder` planner.
It computes a jerk-controlled S-curve profile, then breaks it into time-stepped
segments (up to 32 per move). Each segment is a constant-acceleration slice of the
overall S-curve.

**Smoothieware-CHMT** (firmware) receives these segments and follows the plan using
encoder feedback. It does NOT re-plan the motion — it just executes it.

### OpenPnP → Firmware protocol

OpenPnP sends a sequence like:

```
M920 S12           ← "12 encoder segments incoming, buffer them"
M204 S1500.00      ← acceleration for segment 1
G1 X150.0000 Y200.0000 F3000.00   ← segment 1: target position + feed rate
M204 S1200.00      ← acceleration for segment 2
G1 X155.0000 Y205.0000 F3200.00   ← segment 2
...                ← (repeat for all 12 segments)
```

Key points:
- `M920 S<N>` signals that N segments follow and triggers buffering mode
- Each segment gets its own `F` (feed rate) and `M204 S` (acceleration)
- The F value is the **actual speed for that segment**, not a max — it represents
  the velocity at that point in the S-curve profile
- The M204 acceleration value is `(v_exit - v_entry) / dt` for each segment
- When M920 is not used, normal Smoothieware planning applies (legacy mode)

The M920 emission was added to our OpenPnP fork: https://github.com/c-riegel/openpnp.git

### Firmware M-code reference

| M-code | Purpose |
|--------|---------|
| M918   | Report encoder positions (EX/EY) |
| M919   | Set encoder counters and sync offset |
| M920   | Buffer N segments for S-curve execution |
| M921   | Report stepper step counts (informational only) |
| M922   | Set stepper step counters (debug/sync, idle only) |
| M923   | Set encoder counts per mm |
| M924   | Auto-calibrate encoder counts per mm |
| M925   | Report detailed encoder debug state |

## Current Implementation State

### What works (solid foundation)

- **Hardware encoder interface**: TIM2 (X, PA15/PB3) and TIM5 (Y, PA0/PA1) in
  quadrature mode, 32-bit counters, max input filtering
- **Output Compare ISR target detection**: CC3 channels fire hardware interrupt when
  encoder count matches target — microsecond-latency position detection
- **Three-layer target detection**: OC hardware ISR → step ticker polling fallback →
  on_idle safety net
- **Segment buffering**: M920 triggers buffering, stores up to 128 segments, holds
  the Smoothieware planner queue during execution
- **Planner bypass**: encoder segments run independently through the step ticker
  without going through Smoothieware's block-based planner
- **Safety**: per-move timeouts, halt cleanup, queue discard on completion

### What needs fixing

#### 1. Per-segment speed capture (bug)

**File**: `Encoder.cpp:694`
```cpp
segments[segments_received].feed_rate = THEROBOT->get_feed_rate(); // WRONG
```

`THEROBOT->get_feed_rate()` returns a modal value, not the F from the current G-code
line. Each segment's actual F parameter must be captured from the G-code directly.
The `gcode->get_value('F')` or similar should be used to get the per-segment feed rate.

#### 2. Per-axis speed decomposition (missing)

**File**: `Encoder.cpp:182, 214`

Currently uses a single `feed_rate` for both X and Y:
```cpp
float x_steps_per_sec = (segments[index].feed_rate / 60.0f) * x_stepper->get_steps_per_mm();
```

For diagonal moves, the F value is the hypotenuse velocity. Per-axis speeds are:
```
dx = segment x_target - previous x_target (in mm)
dy = segment y_target - previous y_target (in mm)
dist = sqrt(dx*dx + dy*dy)
theta = atan2(dy, dx)
x_speed = F * cos(theta)   — or equivalently: F * (dx / dist)
y_speed = F * sin(theta)   — or equivalently: F * (dy / dist)
```

This should be **precomputed into the segment struct at buffer time** so the ISR
path does zero floating-point math. The struct needs `x_speed` and `y_speed` fields
(mm/min) or precomputed `x_steps_per_tick` and `y_steps_per_tick` fixed-point values.

#### 3. M204 acceleration not captured (missing)

OpenPnP sends `M204 S<accel>` before each segment. This value is not currently
captured into the segment struct. It may be needed for timeout computation and
could be useful for diagnostics.

#### 4. Post-completion position sync (missing)

After a segment sequence completes, `Robot::machine_position` still reflects the
last G-code target. If the machine didn't perfectly reach the final target (or if
encoder drift accumulated), the Robot's internal position will be wrong. This matters
because the next move (e.g., a small vision-based correction that doesn't use S-curve
segments) will compute its target relative to `machine_position`.

**Fix**: after segments complete, sync `machine_position` from actual encoder position:
```
machine_position[X] = (encoder_x_count / x_counts_per_mm) + x_encoder_offset
machine_position[Y] = (encoder_y_count / y_counts_per_mm) + y_encoder_offset
```

### EncoderSegment struct — current vs needed

Current:
```cpp
struct EncoderSegment {
    int32_t x_target;           // encoder counts
    int32_t y_target;           // encoder counts
    float feed_rate;            // mm/min — WRONG: captures modal F, not per-segment F
    uint32_t timeout_us;        // precomputed
    uint32_t armed_at;          // runtime diagnostic
    uint32_t completed_at;      // runtime diagnostic
    int32_t x_enc_at_arm;       // runtime diagnostic
    int32_t y_enc_at_arm;       // runtime diagnostic
    bool has_x, has_y;
    bool x_skipped, y_skipped;  // DEAD: never set to true
};
```

Needed additions:
```cpp
    float acceleration;         // from M204 S parameter (mm/s²)
    float x_feed_rate;          // precomputed: F * dx/dist (mm/min)
    float y_feed_rate;          // precomputed: F * dy/dist (mm/min)
    // OR precomputed fixed-point for direct ISR use:
    int64_t x_steps_per_tick;   // precomputed for step ticker
    int64_t y_steps_per_tick;   // precomputed for step ticker
```

### Dead code to clean up

- `dbg_x_enc_at_done`, `dbg_y_enc_at_done`, `dbg_x_target_at_done`,
  `dbg_y_target_at_done`, `dbg_x_done_pending`, `dbg_y_done_pending` in Encoder.h
  — declared but never written
- `x_skipped` / `y_skipped` in EncoderSegment — always false
- Unreachable buffering status print at Encoder.cpp ~line 591

## Hardware Notes

- **MCU**: STM32F407
- **Encoder timers**: TIM2 (X axis), TIM5 (Y axis) — both 32-bit
- **OC channels**: CC3 on both timers for target detection
- **us_ticker**: TIM3 — must clear SR flags with single combined write to avoid
  rc_w0 silicon errata (UIF re-set when CC2IF cleared separately)
- **ISR priorities**: OC ISRs at priority 2 (same priority = no preemption race
  between TIM2/TIM5 handlers on Cortex-M4)

## Related Repositories

- **Firmware** (this repo): https://github.com/c-riegel/Smoothieware-CHMT.git
  - Working branch: `feat/encoders`
- **OpenPnP fork** (with M920 emission): https://github.com/c-riegel/openpnp.git
- **Production baseline**: https://git.blueinktech.com/tech/pick-and-place.git
  - `firmware/` folder contains the unmodified Smoothieware fork
