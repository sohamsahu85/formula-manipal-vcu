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
    constexpr uint16_t APPS_DEADBAND_ADC   = 93;     // <= this reads as 0 % pedal
    constexpr uint16_t ADC_FULL            = 1023;   // 10-bit
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
    uint16_t appsADC, bpsADC;      // filtered, rounded
    float    pedalPct;             // pedal % BEFORE any fault cut (for logging)
    float    brakePct;
    float    torqueCmd;            // final command AFTER cuts + slew limit
    bool     bppcCut, canStale, invFault;
    bool     sendBrake;            // true -> adapter sends brake, else current
};

class VcuApp {
public:
    // Fast path: filtering only. No decisions here.
    void sample(uint16_t appsRaw, uint16_t bpsRaw) {
        appsFilt_ = appsFilter_.update(appsRaw);
        bpsFilt_  = bpsFilter_.update(bpsRaw);
    }

    // Command path: deadband -> plausibility -> fault cuts -> slew limit.
    void control(const VcuInputs &in, VcuOutputs &out);

    // Read-only taps on the current filtered signal (post median+EMA), for
    // diagnostics / signal scope only. Available at the 1 kHz sample rate,
    // unlike VcuOutputs which is only refreshed at the 50 Hz command rate.
    float appsFiltered() const { return appsFilt_; }
    float bpsFiltered()  const { return bpsFilt_; }

private:
    static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

    // Arduino map(), reimplemented so it exists without the Arduino macro
    // (arduino_freertos.h #undefs several of those) and so the INTEGER
    // truncation is explicit rather than accidental -- see VcuApp.cpp.
    static long mapInt(long x, long inMin, long inMax, long outMin, long outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    bool bppcCheck(float appsPct, float brakePct);

    MedianEMA<5> appsFilter_{VcuCal::FILTER_ALPHA};
    MedianEMA<5> bpsFilter_ {VcuCal::FILTER_ALPHA};
    float appsFilt_    = 0;
    float bpsFilt_     = 0;
    float appsCmdPrev_ = 0;
    bool  bppcLatched_ = false;
};
