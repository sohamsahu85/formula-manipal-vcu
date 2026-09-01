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
    out.appsADC = (uint16_t)(appsFilt_ + 0.5f);
    out.bpsADC  = (uint16_t)(bpsFilt_  + 0.5f);

    // ---- deadband + scale to pedal travel % ----
    // Reproduces Arduino map() EXACTLY, including its integer truncation (so
    // percentages step 0,1,2...). The validated superloop build used map(), and
    // this must be a faithful extraction -- switching to float division would be
    // a behaviour change disguised as a refactor.
    float apps = (out.appsADC <= VcuCal::APPS_DEADBAND_ADC)
                 ? 0.0f
                 : clampf((float)mapInt(out.appsADC, VcuCal::APPS_DEADBAND_ADC,
                                        VcuCal::ADC_FULL, 0, 100), 0.0f, 100.0f);
    out.pedalPct = apps;                          // pre-cut value, for logging
    out.brakePct = clampf((float)mapInt(out.bpsADC, 0, VcuCal::ADC_FULL, 0, 100),
                          0.0f, 100.0f);

    // ---- APPS/Brake plausibility: judge the REAL pedal, then cut ----
    // (Feeding the already-cut value back in would clear the latch instantly.)
    out.bppcCut = bppcCheck(apps, out.brakePct);
    if (out.bppcCut) apps = 0.0f;

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
