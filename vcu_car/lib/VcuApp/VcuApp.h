/*
 * VcuApp - the VCU control law, with ZERO dependency on any RTOS or hardware.
 * ==========================================================================
 * Pure logic: no analogRead, no CAN calls, no task/mutex/delay. The caller (a
 * "kernel adapter") reads hardware, calls in here, and acts on the outputs.
 *
 * This file lives in shared_libs/ and is compiled into BOTH projects, so the
 * FreeRTOS and miniOS firmwares run byte-identical control logic. That is what
 * makes a head-to-head test compare KERNELS rather than two different programs.
 * Edit it once; both builds pick it up.
 *
 * Call pattern (both adapters do exactly this):
 *      sample(appsRaw, bpsRaw);   // fast rate (1 kHz) - filtering only
 *      control(in, out);          // command rate (50 Hz) - the decisions
 */
#pragma once
#include <Arduino.h>
#include "DTI550.h"
#include "SignalFilter.h"

// ---- calibration / thresholds: single source of truth for both kernels ----
namespace VcuCal {
    constexpr uint16_t ADC_FULL            = 4095;   // 12-bit, 3.3 V ref

    // ---- DUAL-APPS calibration (FSG T11.8): two sensors, DIFFERENT transfer
    // functions, each mapped to pedal % by its OWN endpoints, then compared.
    // MEASURED on the real PCB at 12-bit (apps_cal sweep, 2026-08-29):
    //   APPS1 (A8, 5 V sensor x0.662): rest 0 -> full 3996
    //   APPS2 (A7, 3.3 V sensor x0.675): rest 0 -> full 2587
    // Both rest at 0 counts, so RELEASED=0 gives a true 0 % at rest, no deadzone.
    // NOTE: rest at 0 means a disconnected APPS reads the same as "released" ->
    // low-side open-circuit is NOT distinguishable (rail-to-rail sensors); the
    // dual-sensor cross-check is the main protection.
    constexpr uint16_t APPS1_RELEASED_ADC  = 0;      // A8 pedal up   (measured)
    constexpr uint16_t APPS1_PRESSED_ADC   = 3996;   // A8 pedal down (measured)
    constexpr uint16_t APPS2_RELEASED_ADC  = 0;      // A7 pedal up   (measured)
    constexpr uint16_t APPS2_PRESSED_ADC   = 2587;   // A7 pedal down (measured)
    constexpr uint16_t BPS_PRESSED_ADC     = 3808;   // A6 full brake (measured)

    // FSG T11.8: if the two sensors disagree by >10 pp of pedal travel for
    // >100 ms, motor power must be cut until plausibility returns.
    // We debounce for 80 ms (not the full 100) so that, with the check running
    // at the 50 Hz command rate (20 ms granularity) plus detection latency, the
    // cut always lands at or before the 100 ms ceiling. Detection uses the fast
    // MEDIAN signal (see control()), so this is safe -- a legitimate transient
    // never makes the two median-filtered sensors disagree >10 pp (they track
    // the same pedal with identical lag).
    constexpr float    APPS_IMPLAUS_PCT    = 10.0f;
    constexpr uint32_t APPS_IMPLAUS_MS     = 80;
    constexpr float    BRAKE_ACTUATED_PCT  = 10.0f;  // brake considered "on"
    constexpr float    BPPC_TRIP_PCT       = 25.0f;  // throttle trip while braking
    constexpr float    BPPC_CLEAR_PCT      =  5.0f;  // throttle to clear the latch
    constexpr float    BRAKE_CMD_PCT       =  5.0f;  // above this send brake, not current
    constexpr float    APPS_MAX_RISE       =  5.0f;  // %/tick rising slew limit
    constexpr float    FILTER_ALPHA        =  0.10f; // EMA ~18 Hz at 1 kHz
    constexpr uint32_t CAN_TIMEOUT_MS      = 200;    // stale inverter feedback
}

struct VcuInputs {
    uint32_t    nowMs;
    uint32_t    dtiLastUpdateMs;   // 0 = never heard from the inverter
    DTI550_Data dti;
};

struct VcuOutputs {
    uint16_t appsADC, bpsADC;      // filtered, rounded (appsADC = APPS1)
    uint16_t apps2ADC;             // filtered APPS2, rounded
    float    pedalPct;             // commanded pedal % BEFORE any cut (for logging)
    float    apps1Pct, apps2Pct;   // each sensor mapped by its own calibration
    float    brakePct;
    float    torqueCmd;            // final command AFTER cuts + slew limit
    bool     bppcCut, canStale, invFault;
    bool     appsImplaus;          // T11.8: the two APPS disagree (latched)
    bool     sendBrake;            // true -> adapter sends brake, else current
};

class VcuApp {
public:
    // Fast path: filtering only. No decisions here. Now DUAL-APPS: both
    // accelerator sensors plus the brake are filtered every call.
    void sample(uint16_t apps1Raw, uint16_t apps2Raw, uint16_t bpsRaw) {
        appsFilt_  = appsFilter_.update(apps1Raw);
        apps2Filt_ = apps2Filter_.update(apps2Raw);
        bpsFilt_   = bpsFilter_.update(bpsRaw);
        // Median-only taps for the fast T11.8 plausibility check (no EMA lag).
        apps1Med_  = appsFilter_.median();
        apps2Med_  = apps2Filter_.median();
    }

    // Command path: map each APPS -> T11.8 implausibility -> BPPC -> fault cuts
    // -> slew limit.
    void control(const VcuInputs &in, VcuOutputs &out);

    // Read-only taps on the current filtered signals (post median+EMA), for
    // diagnostics / signal scope only. Available at the 1 kHz sample rate,
    // unlike VcuOutputs which is only refreshed at the 50 Hz command rate.
    float appsFiltered()  const { return appsFilt_; }
    float apps2Filtered() const { return apps2Filt_; }
    float bpsFiltered()   const { return bpsFilt_; }

private:
    static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

    // Arduino map(), reimplemented so it exists without the Arduino macro
    // (arduino_freertos.h #undefs several of those) and so the INTEGER
    // truncation is explicit rather than accidental -- see VcuApp.cpp.
    static long mapInt(long x, long inMin, long inMax, long outMin, long outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    bool bppcCheck(float appsPct, float brakePct);

    MedianEMA<5> appsFilter_ {VcuCal::FILTER_ALPHA};
    MedianEMA<5> apps2Filter_{VcuCal::FILTER_ALPHA};
    MedianEMA<5> bpsFilter_  {VcuCal::FILTER_ALPHA};
    float    appsFilt_     = 0;
    float    apps2Filt_    = 0;
    float    bpsFilt_      = 0;
    float    apps1Med_     = 0;        // median-only (fast) for T11.8 detection
    float    apps2Med_     = 0;
    float    appsCmdPrev_  = 0;
    bool     bppcLatched_  = false;
    uint32_t implausSince_ = 0;        // ms when disagreement began (0 = plausible)
    bool     implausLatched_ = false;  // T11.8 cut active
};
