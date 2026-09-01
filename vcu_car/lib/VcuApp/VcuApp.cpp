#include "VcuApp.h"

/*
 * APPS / Brake plausibility (FSAE 25 %/5 % pattern; defense-in-depth, NOT a
 * substitute for the hardware BSPD -- FSG T 11.6 requires that separately).
 *
 * Latches when the brake is actuated AND throttle > 25 %. Once latched the
 * accelerator stays cut until throttle drops below 5 % -- REGARDLESS of whether
 * the brake is still pressed. The two different thresholds are deliberate
 * hysteresis; with a single value it would chatter around the threshold.
 */
bool VcuApp::bppcCheck(float appsPct, float brakePct) {
    if (brakePct > VcuCal::BRAKE_ACTUATED_PCT && appsPct > VcuCal::BPPC_TRIP_PCT)
        bppcLatched_ = true;                      // trip
    if (appsPct < VcuCal::BPPC_CLEAR_PCT)
        bppcLatched_ = false;                     // clear (brake deliberately not consulted)
    return bppcLatched_;
}

void VcuApp::control(const VcuInputs &in, VcuOutputs &out) {
    out.appsADC  = (uint16_t)(appsFilt_  + 0.5f);
    out.apps2ADC = (uint16_t)(apps2Filt_ + 0.5f);
    out.bpsADC   = (uint16_t)(bpsFilt_   + 0.5f);

    // ---- map EACH APPS to pedal % by its own calibration (no fixed deadband) ----
    // The two sensors have deliberately different transfer functions (FSG
    // T11.8.6), so each has its own released/pressed endpoints.
    out.apps1Pct = clampf((float)mapInt(out.appsADC,  VcuCal::APPS1_RELEASED_ADC,
                                        VcuCal::APPS1_PRESSED_ADC, 0, 100), 0.0f, 100.0f);
    out.apps2Pct = clampf((float)mapInt(out.apps2ADC, VcuCal::APPS2_RELEASED_ADC,
                                        VcuCal::APPS2_PRESSED_ADC, 0, 100), 0.0f, 100.0f);

    // ---- FSG T11.8 dual-APPS implausibility ----
    // Detection runs on the MEDIAN signal (spike-rejected but NOT EMA-smoothed)
    // so it reacts within ~1 control cycle -- the EMA output would add ~20 ms of
    // threshold-crossing lag and push total detection over the 100 ms deadline.
    // (The EMA-based apps1Pct/apps2Pct above are still used for the torque
    // command and telemetry, where smoothness matters.)
    float med1Pct = clampf((float)mapInt((long)(apps1Med_ + 0.5f),
                        VcuCal::APPS1_RELEASED_ADC, VcuCal::APPS1_PRESSED_ADC, 0, 100), 0.0f, 100.0f);
    float med2Pct = clampf((float)mapInt((long)(apps2Med_ + 0.5f),
                        VcuCal::APPS2_RELEASED_ADC, VcuCal::APPS2_PRESSED_ADC, 0, 100), 0.0f, 100.0f);
    // If the two disagree by >10 pp for >100 ms, cut motor power. Latches while
    // implausible; clears when they agree again. The timer enforces ">100 ms" --
    // a brief glitch does not trip.
    float deviation = med1Pct - med2Pct;
    if (deviation < 0) deviation = -deviation;
    if (deviation > VcuCal::APPS_IMPLAUS_PCT) {
        if (implausSince_ == 0) implausSince_ = in.nowMs;          // start timing
        else if ((uint32_t)(in.nowMs - implausSince_) >= VcuCal::APPS_IMPLAUS_MS)
            implausLatched_ = true;                                // debounce elapsed -> cut
    } else {
        implausSince_    = 0;                                      // plausible again
        implausLatched_  = false;                                 // power may resume
    }
    out.appsImplaus = implausLatched_;

    // Commanded pedal = the lower of the two (conservative when they agree).
    float apps = (out.apps1Pct < out.apps2Pct) ? out.apps1Pct : out.apps2Pct;
    out.pedalPct = apps;                          // pre-cut value, for logging

    out.brakePct = clampf((float)mapInt(out.bpsADC, 0, VcuCal::BPS_PRESSED_ADC, 0, 100),
                          0.0f, 100.0f);

    // ---- APPS/Brake plausibility (BPPC): judge the REAL pedal, then cut ----
    // (Feeding the already-cut value back in would clear the latch instantly.)
    out.bppcCut = bppcCheck(apps, out.brakePct);
    if (out.bppcCut) apps = 0.0f;

    // ---- T11.8 dual-APPS implausibility cut ----
    if (out.appsImplaus) apps = 0.0f;

    // ---- motor-controller faults: never drive into a faulted/silent inverter ----
    out.canStale = (in.dtiLastUpdateMs == 0) ||   // never heard from it = unsafe
                   ((uint32_t)(in.nowMs - in.dtiLastUpdateMs) > VcuCal::CAN_TIMEOUT_MS);
    out.invFault = (in.dti.Actual_FaultCode != 0);
    if (out.canStale || out.invFault) apps = 0.0f;

    // ---- rising slew limit; a cut passes straight through (safe direction) ----
    float d = apps - appsCmdPrev_;
    if (d > VcuCal::APPS_MAX_RISE) d = VcuCal::APPS_MAX_RISE;
    apps = appsCmdPrev_ + d;
    appsCmdPrev_ = apps;

    out.torqueCmd = apps;
    out.sendBrake = (out.brakePct > VcuCal::BRAKE_CMD_PCT);
}
