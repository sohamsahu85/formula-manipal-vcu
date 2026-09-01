/*
 * VcuSelfTest - deterministic input script for the kernel comparison (Phase 3).
 * ===========================================================================
 * Feeds a FIXED sequence of pedal / fault inputs through VcuApp and reports the
 * outputs. Both the FreeRTOS and miniOS adapters run this identical routine, so
 * the output MUST be byte-for-byte identical on both kernels.
 *
 * If it isn't, the difference cannot be the control logic (same VcuApp) or the
 * input (same script) -- it can only be the KERNEL: FPU context save/restore
 * corrupting a float across preemption, or the command cadence running at the
 * wrong rate and shifting the slew limiter. That is exactly the comparison
 * Phase 3 exists to make.
 *
 * The script deliberately exercises every branch of control():
 *   - deadband + rest
 *   - slew-limited ramp up (torque must rise <=5 %/step)
 *   - saturation at full
 *   - instant release (a drop is NOT slew limited)
 *   - BPPC trip (brake + >25 % throttle), latch hold, and clear (<5 %)
 *   - CAN stale (feedback older than the timeout)
 *   - inverter fault code
 *
 * Uses its OWN VcuApp instance, so running it never disturbs the live
 * controller's filter or latch state.
 */
#pragma once
#include "VcuApp.h"

struct VcuTestStep {
    uint16_t apps;      // raw APPS ADC (0..1023)
    uint16_t bps;       // raw BPS ADC
    uint8_t  fault;     // inverter fault code (0 = none)
    uint16_t ageMs;     // age of last CAN feedback: 0 = fresh, >timeout = stale
    const char *note;   // what this step is checking
};

// The script, at the 50 Hz command cadence (20 ms each). Phases are HELD long
// enough for the median+EMA filters to settle -- a single-step input barely
// moves the filtered value, so a branch like BPPC (which keys off filtered
// brake %) only trips after the brake has been held for several steps.
static const VcuTestStep VCU_TEST_SCRIPT[] = {
    // --- rest / deadband (torque must stay 0) ---
    {   0,   0, 0, 0, "rest" },
    {   0,   0, 0, 0, "rest" },
    {  50,   0, 0, 0, "below deadband" },
    {  93,   0, 0, 0, "at deadband edge" },
    // --- slew-limited ramp to full: torque must rise <=5 %/step, ~20 steps ---
    { 1023,  0, 0, 0, "step to full" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "saturated" }, { 1023, 0, 0, 0, "saturated" },
    // --- instant release: torque drops to 0 in ONE step (no slew on fall) ---
    {   0,   0, 0, 0, "release (instant drop)" },
    {   0,   0, 0, 0, "rest" },
    // --- BPPC trip: hold throttle up (~10 steps), then hold brake ON so the
    //     BPS filter climbs past 10 % while throttle is > 25 % ---
    { 1023,  0, 0, 0, "throttle up" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    { 1023, 900, 0, 0, "brake on" }, { 1023, 900, 0, 0, "brake settling" },
    { 1023, 900, 0, 0, "brake settling" }, { 1023, 900, 0, 0, "brake settling" },
    { 1023, 900, 0, 0, "brake settling" }, { 1023, 900, 0, 0, "TRIP expected" },
    { 1023, 900, 0, 0, "latched" }, { 1023, 900, 0, 0, "latched" },
    // --- latch holds: brake released but throttle still high ---
    { 1023,  0, 0, 0, "brake off, throttle held -> STILL CUT" },
    { 1023,  0, 0, 0, "still latched" }, { 1023, 0, 0, 0, "still latched" },
    { 1023,  0, 0, 0, "still latched" }, { 1023, 0, 0, 0, "still latched" },
    // --- clear: throttle to 0 and HELD ~28 steps. The filtered APPS decays at
    //     EMA alpha 0.1 (~0.9x/step), so it takes ~25 steps to fall under the
    //     deadband and drive pedal % below the 5 % that clears the latch. ---
    {   0,   0, 0, 0, "throttle off" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "settling" },
    {   0,   0, 0, 0, "settling" }, { 0, 0, 0, 0, "CLEAR expected" },
    // --- throttle back up, no latch, ramp resumes ---
    { 1023,  0, 0, 0, "throttle back" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" }, { 1023, 0, 0, 0, "ramp" },
    // --- CAN stale: feedback older than the 200 ms timeout cuts torque ---
    { 1023,  0, 0, 400, "CAN stale -> cut" }, { 1023, 0, 0, 400, "still stale" },
    { 1023,  0, 0, 400, "still stale" },
    // --- inverter fault code cuts torque ---
    { 1023,  0, 3, 0, "inverter fault -> cut" }, { 1023, 0, 3, 0, "still faulted" },
    // --- everything clears, ramp resumes from 0 ---
    { 1023,  0, 0, 0, "clear (ramp resumes)" }, { 1023, 0, 0, 0, "ramp" },
    { 1023,  0, 0, 0, "ramp" },
    {   0,   0, 0, 0, "final rest" },
};
static const int VCU_TEST_STEPS =
    (int)(sizeof(VCU_TEST_SCRIPT) / sizeof(VCU_TEST_SCRIPT[0]));

// Runs the script through `app` and calls sink() for each row. Deterministic:
// one sample()+control() per step, 20 ms of simulated time per step, so the
// slew limiter and BPPC latch advance identically on any kernel.
typedef void (*VcuTestSink)(int idx, const VcuTestStep &in, const VcuOutputs &out);

inline void vcuSelfTest(VcuApp &app, VcuTestSink sink) {
    uint32_t now = 100000;                      // arbitrary base time (ms)
    for (int i = 0; i < VCU_TEST_STEPS; i++) {
        const VcuTestStep &s = VCU_TEST_SCRIPT[i];
        now += 20;                              // 50 Hz command cadence

        app.sample(s.apps, s.bps);

        DTI550_Data dti{};
        dti.Actual_FaultCode = s.fault;
        // ageMs == 0 -> feedback is fresh (updated 'now'); else it is that old.
        uint32_t lastUpd = (s.ageMs == 0) ? now : (now - s.ageMs);

        VcuInputs  in{ now, lastUpd, dti };
        VcuOutputs out{};
        app.control(in, out);

        sink(i, s, out);
    }
}
