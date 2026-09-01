# Formula Manipal — VCU & Custom RTOS

Vehicle Control Unit (VCU) firmware for a Formula Student electric racecar, built
on a **Teensy 4.1** commanding a **DTI inverter over CAN**. This repository also
contains **miniOS**, a from-scratch preemptive real-time kernel, and a rigorous
head-to-head validation of it against FreeRTOS running identical control logic.

> Competition ruleset: **Formula Student Germany (FSG)**. The firmware here is
> development/bench work; anything touching high voltage is gated behind the
> checks described below.

## Layout

| Path | What it is |
|------|------------|
| `shared_libs/` | Kernel-agnostic control logic (`VcuApp`), CAN decode (`DTI550`), filtering (`SignalFilter`), SD logging (`DataLogger`). No RTOS or hardware calls — pure functions. |
| `vcu_minios/` | The VCU on **miniOS** (custom kernel in `lib/miniOS`). |
| `vcu_freertos/` | The VCU on **FreeRTOS** — the trusted reference. |
| `vcu_car/` | The **car firmware**: self-contained miniOS build for the real PCB, with dual-APPS plausibility and measured pedal calibration. |
| `phase3_results/` | Evidence from the kernel validation (byte-for-byte output match, load timing, 30-min soak). |
| `apps_cal/` | Standalone APPS/BPS bring-up + calibration reader. |

The single-copy rule is the point: `shared_libs` is compiled into **both** kernel
builds, so a head-to-head test compares *kernels*, not two drifting programs.

## miniOS — the custom kernel

A ~500-line fixed-priority preemptive kernel for the Cortex-M7F:

- PendSV context switch with **FPU (S16–S31) save/restore**
- Priority scheduling, real blocking (`osSleep`/`osSleepUntil`), round-robin among equals
- **Priority-inheritance** mutex (measured 121 ms vs ~290 ms without, on hardware)
- Fully static allocation; PIT tick (keeps Arduino `millis()` working)
- Survivability: stack canaries + high-water marks, named fault reporting, cooperative watchdog

## Kernel validation (Phase 3)

miniOS was validated against FreeRTOS on real hardware. See `phase3_results/`:

1. **Same input → same output** — a fixed 88-step script through the shared
   `VcuApp` produced **byte-for-byte identical** output on miniOS, FreeRTOS, and a
   PC reference (every torque value to 3 decimals).
2. **Deadlines under load** — both held a flat 1 kHz control loop with CAN + SD +
   Serial running; no dips.
3. **30-minute soak** — no drift, no reboot, bounded stack (high-water plateaued).

Two honest findings (about FreeRTOS's *maturity*, not miniOS being wrong): FreeRTOS
avoids a `wfi`/`millis()` idle-clock trap by default, and has a tighter timebase.
Conclusion: **ship FreeRTOS under the throttle; miniOS is a validated research
artifact** that demonstrates, from the inside, what the car's OS actually does.

## Control law (`VcuApp`)

Runs the same on either kernel:

- 1 kHz sampling → median-of-5 (EMI spike rejection) → EMA → 50 Hz command
- **Dual-APPS plausibility (FSG T11.8):** two sensors with different transfer
  functions; >10 pp disagreement for >~80 ms cuts torque (detection on the fast
  median path to meet the 100 ms deadline)
- **APPS/Brake plausibility (BPPC):** latching cut on brake + throttle
- Fault cuts on inverter fault / stale CAN; rising-only slew limit (cuts are instant)

## Build

Each project is a [PlatformIO](https://platformio.org/) project:

```bash
cd vcu_car        # or vcu_minios, vcu_freertos, apps_cal
pio run           # build
pio run -t upload # flash a Teensy 4.1
```

## Status

Bench/development firmware. High-voltage operation additionally requires the
deferred safety scaffolding (R2D state machine, acting watchdog, safe-start
guard) and on-vehicle calibration — see the notes in `vcu_car/`.
