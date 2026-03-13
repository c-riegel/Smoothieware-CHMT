# Smoothieware-CHMT: Closed-Loop Encoder Control

## Core Principle — READ THIS FIRST

This project converts Smoothieware from open-loop stepper control to **closed-loop encoder-based control** for the X and Y axes of a CHMT pick-and-place machine.

**The encoder is the ONLY source of truth for position. NEVER use step counts for positioning.**

Step counts are informational at best. They have no usable purpose for position tracking,
transition detection, or motion decisions. The whole point of this project is that steppers
skip steps under load/speed — so step counts are unreliable by definition.

If you find yourself writing code that references step counts for anything position-related,
STOP. You are going down the wrong path.

## When You Get Stuck

When something seems "non-trivial" or you feel tempted to mix step counts with encoder data
to solve a problem, **flag it to the user immediately** instead of implementing a hybrid
approach. The user can show you the simpler encoder-only solution. Do not try to engineer
around the closed-loop principle — there is always an easier path that preserves it.

## Motion Planning Architecture

- **OpenPnP** does all path planning using its Simulated3rdOrder motion planner
- OpenPnP sends up to 32 segments per move — this IS the motion plan
- The firmware's job is to **follow the precomputed OpenPnP plan faithfully**
- Smoothieware's built-in planner must be **bypassed** for encoder segments

### What this means concretely:

- Do NOT let Smoothieware add its own ramp-up/ramp-down to OpenPnP segments
- Do NOT ramp down to "stopped" between segments — maintain speed through transitions
- Transition between segments at current speed, adjusting to the next segment's target speed
- Use encoder data to detect when a transition point is reached
- Cache per-segment data: encoder-based X/Y transition points, target speed, etc.

### Two operating modes:

1. **OpenPnP precomputed mode** (triggered by M-codes): bypass Smoothieware's planner,
   execute encoder-driven segments directly
2. **Legacy mode** (no M-codes): use Smoothieware's existing planner as-is

## Current WIP State (2026-03-13)

- Encoder segment mode with planner bypass is implemented
- Per-segment speed: firmware captures F from each G1, decomposes per-axis via
  `F * |dx|/dist`, precomputes `x_steps_per_tick` / `y_steps_per_tick` (2.62 fixed-point)
- OpenPnP fork has "Per-Segment Feed Rate?" option that emits correct F per segment
  and suppresses M204 (which caused a combined-line parsing bug)
- Position sync after segments: `THEROBOT->reset_axis_position()` from encoder data
- **Need to rebuild and flash** — last flash used stale padded binary

## Building and Flashing

- Build: `make -j4` from repo root
- Flash: `make stm32-flash` (pads to 512KB and flashes via st-flash)
- ST-Link V3: first flash attempt often fails, retry immediately
- During flash erase, GPIOs float — power cycle after flashing

## OpenPnP Companion Repo

The OpenPnP fork at `github.com/c-riegel/openpnp` (local: `~/Documents/git/openpnp-production/`)
has custom features for this firmware:
- **M920 segment buffering** (`feat/m920-segment-buffering`)
- **Per-segment feed rate** (`feat/per-segment-feed-rate-v2`)
- Build: `mvn package -DskipTests`
- Run for testing: `java -DconfigDir=$HOME/.openpnp2-test -jar target/openpnp-gui-0.0.1-alpha-SNAPSHOT.jar 2>&1 | tee ~/Documents/git/openpnp_debug.txt`
- Test config: `$HOME/.openpnp2-test` (separate from production `$HOME/.openpnp2`)
- Debug log: `~/Documents/git/openpnp_debug.txt`

## Hardware Notes

- STM32F407 platform
- us_ticker TIM3 ISR: must clear SR flags with a single combined write to avoid
  rc_w0 silicon errata that re-sets UIF when CC2IF is cleared separately
- X and Y axes have encoders; encoder data drives all position logic
- X encoder: PA_15 + PB_3 on TIM2, Y encoder: PA_0 + PA_1 on TIM5

## Firmware M-code Reference

| M-code | Purpose |
|--------|---------|
| M918   | Report encoder positions (EX/EY) |
| M919   | Set encoder counters and sync offset |
| M920   | Buffer N segments for S-curve execution |
| M921   | Report stepper step counts (informational only) |
| M922   | Set stepper step counters (debug/sync, idle only) |
| M923   | Set encoder counts per mm (activates encoder control) |
| M924   | Auto-calibrate encoder counts per mm |
