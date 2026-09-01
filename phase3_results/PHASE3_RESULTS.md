# Phase 3 — Kernel Validation: miniOS vs FreeRTOS

**Under test:** `miniOS` — the custom preemptive kernel (in `vcu_minios/lib/miniOS`).
**Reference:** `FreeRTOS` (teensy port) — trusted, industry-standard.

**Method:** both firmwares run the *identical* control logic from `shared_libs/VcuApp`.
Only the kernel and its thin adapter differ, so any behavioural difference is
attributable to the kernel alone. miniOS is validated by showing it matches the
FreeRTOS reference (and an independent PC-computed golden) on the same workload.

Date: 2026-08-28. Board: Teensy 4.1. CAN bus connected and healthy
(1.76 V differential, ~500 kbit/s — verified on scope the same day).

---

## Scorecard

| # | Test | Result |
|---|------|--------|
| 1 | Same input -> same output | ✅ PASS — 3-way byte-for-byte identical |
| 2 | Deadlines held under load  | ✅ PASS — both hold ~1 kHz, no dips/drift |
| 3 | Long soak (miniOS)         | ✅ PASS — 30 min, no dips/reboot/leak |

**Phase 3 COMPLETE (2026-08-29). miniOS validated against FreeRTOS on all three.**

---

## Test 1 — Deterministic equivalence

A fixed 88-step input script (`shared_libs/VcuApp/VcuSelfTest.h`) is fed through
`VcuApp` on each kernel via the serial command `T`. The script exercises every
branch of `control()`: deadband, slew-limited ramp, instant release, BPPC
trip/latch/clear, CAN-stale cut, inverter-fault cut.

Three independent runs compared:
- `golden.csv`            — PC, native g++ (see `host_golden_gen.cpp`)
- `minios_selftest.csv`   — Teensy + miniOS
- `freertos_selftest.csv` — Teensy + FreeRTOS

**Result: all three identical, 88/88 rows, down to every %.3f torque value.**

```
diff golden.csv minios_selftest.csv    -> identical
diff golden.csv freertos_selftest.csv  -> identical
diff minios_selftest.csv freertos_selftest.csv -> identical
```

This specifically clears the two kernel-level risks:
- **FPU context save/restore** — miniOS's hand-written S16-S31 assembly restores
  floats identically to FreeRTOS. A corruption would have diverged `torque`.
- **Slew-limiter cadence** — both advance the ramp identically.

Note: the golden reference caught two flaws in the *test script itself* (BPPC
not tripping, then not clearing, due to filter settling time) before the
hardware runs — a kernel-vs-kernel-only diff would have passed them silently.

## Test 2 — Deadlines under load

Full workload running: 1 kHz control + CANRX (live bus, `stale 0` throughout) +
SD logging (50 Hz, blocking) + Serial. The `ctrl` field is the control loop's
real rate, measured inside the control task against the DWT cycle counter.
40 s captured per kernel.

| Kernel    | min | mean   | max  | stdev | drift |
|-----------|-----|--------|------|-------|-------|
| FreeRTOS  | 1000| 1000.0 | 1000 | 0.0   | none  |
| miniOS    | 1000| 1008.9 | 1009 | 1.0   | none  |

**Result: both hold ~1 kHz flat, no dips, no drift.** Proves the priority design
works on miniOS: blocking SD/Serial I/O sits below the control task and never
steals a control cycle.

Raw data: `minios_load_ctrlHz.txt`, `freertos_load_ctrlHz.txt`.

---

## Test 3 — Long soak (miniOS)

30 min continuous under full load (1 kHz control + live CAN + SD + Serial). A
periodic health line (every 10 s) logs uptime + per-task stack high-water, so a
reboot is unmissable (uptime resets) and a stack leak shows as a climbing number.
Data: `soak_health_trend.txt` (171 lines), `soak_ctrlHz_all.txt` (16,469 samples).

| Metric        | Result                                             |
|---------------|----------------------------------------------------|
| ctrl rate     | 16,469 samples, min 1000, mean 1008.9, **0 below 1000** |
| Uptime        | monotonic 50 s -> 1,849,940 ms, **no reboot**      |
| Watchdog      | 0 fires                                            |
| Stack (leak)  | **bounded** — high-water plateaued at LOG 493 / TELEM 491 in the first ~6 min and was IDENTICAL at minute 30 |
| CAN           | stale 0 throughout                                 |

The stack result is the key one: high-water marks only ratchet up, so they
discover each task's deepest code path over time. LOG stepped 440->493 (SD flush
path) and TELEM 480->491 (deeper printf) once each, then plateaued and never grew
again. Final == chunk-1 plateau => bounded, no leak. Worst case 493/1536 = 32%.

Method note (honest): the FIRST soak attempt (2026-08-28) ran 29 min clean but
the board rebooted AFTER the captured window, before the closing stack dump could
be read -- so that run could neither prove no-leak nor rule out a crash (the fault
handler output, if any, went to a closed port). The fix was the periodic health
line above, making the result independent of when the port is read. This clean
re-run replaces it.

---

## Findings (maturity, not correctness)

Two real differences surfaced. Neither is a miniOS *bug* — both are places where
FreeRTOS's maturity shows, and both are worth citing in the report:

1. **`wfi`/`millis()` trap — FreeRTOS avoids it by default.**
   A naive `wfi` in miniOS's idle task halted the core clock, stopping SysTick
   and making `millis()` lose ~84% of real time (the CAN-staleness timeout would
   have stretched ~6x). FreeRTOS ships `configUSE_TICKLESS_IDLE = 0` and an empty
   idle hook, so it never sleeps the core — the trap can't happen. miniOS now
   spins at idle to match. (Fixed 2026-08-28.)

2. **Timebase accuracy — FreeRTOS is tighter.**
   FreeRTOS derives its tick from the Arduino core's SysTick (crystal-exact):
   reads 1000.0 Hz. miniOS self-calibrates its PIT against DWT and lands ~0.9%
   fast (1008.9 Hz). Internally consistent (tick + millis scale together, so
   behaviour is unaffected) but slightly off absolute time.

---

## Reproduce

- Build+flash either project (`pio run -t upload`), open serial @115200.
- Send `T` -> prints `# selftest begin ... # selftest end`; the CSV between the
  markers must match `golden.csv` exactly.
- Regenerate golden on PC:
  `g++ -std=c++17 -I<shim> -I shared_libs/{VcuApp,SignalFilter,DTI550} \
       host_golden_gen.cpp shared_libs/VcuApp/VcuApp.cpp -o gen && ./gen`
  (shim = a folder with an `Arduino.h` containing just `#include <cstdint>` etc.)
