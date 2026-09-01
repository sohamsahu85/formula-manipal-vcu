/*
 * Formula Manipal VCU - miniOS ADAPTER  -  Teensy 4.1
 * ===================================================
 * Same ../shared_libs/VcuApp control logic as the FreeRTOS build. Only the
 * kernel and this adapter differ, so any behavioural divergence between the two
 * firmwares is a KERNEL issue, not a logic issue.
 *
 * Task table (mirrors the FreeRTOS build's ordering)
 *   controlTask    prio 7  1 kHz   sample -> (50 Hz) control -> CAN TX
 *   canRxTask      prio 6  poll    drain CAN -> DTI550_decode()
 *   watchdogTask   prio 5  10 Hz   cooperative check-in monitor
 *   logTask        prio 2  50 Hz   SD writes (blocking I/O, low priority)
 *   telemetryTask  prio 1  10 Hz   Serial (lowest)
 *
 * Two deliberate differences from the FreeRTOS adapter, and why:
 *   - miniOS has no queues yet, so Control hands off a mutex-guarded snapshot
 *     instead. Same "never block the control path" property: Control uses a
 *     0-timeout take and simply skips the handoff if the mutex is busy.
 *   - miniOS mutexes can spuriously fail (documented Phase-1 limitation), so
 *     every take is best-effort with a known-good fallback.
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "miniOS.h"
#include "DTI550.h"
#include "VcuApp.h"
#include "VcuSelfTest.h"
#include "DataLogger.h"

void DTI550_decode(const CAN_message_t &msg);

// ---------------- Pins / rates ----------------
#define APPS_PIN A2
#define BPS_PIN  A1
static const uint32_t CMD_DIVIDER  = 20;     // 1 kHz / 20 -> 50 Hz
static const uint32_t TELEMETRY_MS = 100;
static const uint32_t WATCHDOG_MS  = 100;

// ---------------- DTI command frames (identical to the FreeRTOS adapter) ----
static const uint8_t NODE            = 4;
static const uint8_t PID_REL_CURRENT = 0x05;
static const uint8_t PID_REL_BRAKE   = 0x06;
static const uint8_t PID_DRIVE_EN    = 0x0C;
static inline bool isDtiFeedback(uint32_t id) {
    return id == 0x404 || id == 0x424 || id == 0x444 || id == 0x464 || id == 0x484;
}
static CAN_message_t cmdFrame(uint8_t pid) {
    CAN_message_t m = {};
    m.id = (pid << 5) | NODE;
    m.len = 8;
    for (int i = 0; i < 8; i++) m.buf[i] = 0xFF;
    return m;
}
static CAN_message_t dtiScaled(uint8_t pid, float pct) {
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    int16_t v = (int16_t)(pct * 10.0f);
    CAN_message_t m = cmdFrame(pid);
    m.buf[0] = (uint8_t)(v >> 8);
    m.buf[1] = (uint8_t)(v & 0xFF);
    return m;
}
static CAN_message_t dtiDriveEnable(bool en) {
    CAN_message_t m = cmdFrame(PID_DRIVE_EN);
    m.buf[0] = en ? 1 : 0;
    return m;
}

// ---------------- Shared objects ----------------
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
static VcuApp     app;                      // <-- the shared control logic
static DataLogger logger;

static OsMutex dtiMtx, canMtx, snapMtx;
static volatile uint32_t dtiLastUpdateMs = 0;

struct Snapshot { VcuOutputs out; DTI550_Data dti; uint32_t t_ms; };
static Snapshot      shared;                // Control -> Log / Telemetry
static volatile bool sharedFresh = false;   // set by Control, cleared by Log

// Measured control-loop rate, target 1000 Hz. Printed alongside the kernel tick
// rate and millis() so the three can be cross-checked: a wrong reading in only
// one of them localises the fault (loop vs kernel tick vs SysTick).
static volatile uint32_t ctrlLoops = 0;

// ===================== signal scope (bench diagnostic) =====================
/*
 * A software oscilloscope on the APPS/BPS chain, tapped exactly where the
 * controller reads it -- raw ADC and the median+EMA output, side by side. This
 * is how you SEE whether the pedal filtering is earning its place instead of
 * guessing: arm a capture, wiggle the pedal (or inject EMI), and compare traces.
 *
 * OFF by default. The capture path is a few array writes guarded by a flag, run
 * inside the 1 kHz control task -- it does NOT touch the control decision, and
 * costs nothing when idle. All output happens in the low-priority telemetry
 * task, so scoping can never delay a torque command.
 *
 * Console commands (see telemetryTask): c=capture  l=live  x=stop  t=telemetry
 */
static const uint16_t SCOPE_N = 1000;        // single-shot depth: 1 s @ 1 kHz
struct ScopeRow { uint16_t rawA, filtA, rawB, filtB; };
static ScopeRow          scopeBuf[SCOPE_N];
static volatile uint16_t scopeIdx   = 0;
static volatile bool     scopeArmed = false; // filling the single-shot buffer
static volatile bool     scopeReady = false; // buffer full, ready to dump
static volatile bool     scopeLive  = false; // continuous Serial-Plotter stream
// latest sample for the live stream (16-bit stores are atomic on Cortex-M)
static volatile uint16_t liveRawA, liveFiltA, liveRawB, liveFiltB;

// Called every control tick with the values as the controller actually sees
// them. Full-rate (1 kHz) capture is what lets it catch single EMI spikes that
// a decimated print would step right over.
static inline void scopeSample(uint16_t rawA, float filtA,
                               uint16_t rawB, float filtB) {
    uint16_t fA = (uint16_t)(filtA + 0.5f), fB = (uint16_t)(filtB + 0.5f);
    liveRawA = rawA; liveFiltA = fA;
    liveRawB = rawB; liveFiltB = fB;
    if (scopeArmed && scopeIdx < SCOPE_N) {
        scopeBuf[scopeIdx] = { rawA, fA, rawB, fB };
        if (++scopeIdx >= SCOPE_N) { scopeArmed = false; scopeReady = true; }
    }
}

// ===================== canRxTask (prio 6) =====================
static void canRxTask(void *) {
    osWatchdogRegister();
    CAN_message_t rx;
    for (;;) {
        bool got = false;
        if (osMutexTake(&canMtx, 2)) { got = Can2.read(rx); osMutexGive(&canMtx); }
        if (got) {
            if (osMutexTake(&dtiMtx, 2)) { DTI550_decode(rx); osMutexGive(&dtiMtx); }
            if (isDtiFeedback(rx.id)) dtiLastUpdateMs = millis();
        } else {
            osSleep(1);
        }
        osTaskCheckIn();
    }
}

// ===================== controlTask (prio 7) =====================
static void controlTask(void *) {
    osWatchdogRegister();

    // Trim the kernel tick to exactly 1 kHz before any control decision is made.
    // Everything downstream -- the 50 Hz divider, the slew limit, every timeout
    // -- is denominated in ticks, so a few percent here shifts all of it.
    osTickCalibrate(1000);

    uint32_t    div = 0;
    DTI550_Data dtiCopy{};                  // last known-good feedback
    uint32_t    nextWake = osTickCount();   // absolute deadline (drift-free)

    for (;;) {
        // --- 1 kHz: filtering only ---
        uint16_t rawA = analogRead(APPS_PIN);
        uint16_t rawB = analogRead(BPS_PIN);
        app.sample(rawA, rawB);
        scopeSample(rawA, app.appsFiltered(), rawB, app.bpsFiltered());

        // --- 50 Hz: the control decision ---
        if (++div >= CMD_DIVIDER) {
            div = 0;

            // best-effort: keep the previous copy rather than stall the safety path
            if (osMutexTake(&dtiMtx, 1)) { dtiCopy = dti550; osMutexGive(&dtiMtx); }

            VcuInputs  in{ millis(), dtiLastUpdateMs, dtiCopy };
            VcuOutputs out{};
            app.control(in, out);           // <-- identical call in both kernels

            if (osMutexTake(&canMtx, 2)) {
                Can2.write(dtiDriveEnable(true));
                Can2.write(out.sendBrake ? dtiScaled(PID_REL_BRAKE, out.brakePct)
                                         : dtiScaled(PID_REL_CURRENT, out.torqueCmd));
                osMutexGive(&canMtx);
            }

            // hand off to the low-priority tasks; skip if busy (never block)
            if (osMutexTake(&snapMtx, 0)) {
                shared = Snapshot{ out, dtiCopy, millis() };
                sharedFresh = true;
                osMutexGive(&snapMtx);
            }
        }

        ctrlLoops++;                        // for the measured-rate readout
        osTaskCheckIn();
        // Drift-free 1 kHz: osSleep(1) would give 1 ms PLUS the work time, so the
        // period would stretch by however long the cycle took.
        osSleepUntil(&nextWake, 1);
    }
}

// ===================== logTask (prio 2) =====================
static void logTask(void *) {
    for (;;) {
        Snapshot s; bool have = false;
        if (osMutexTake(&snapMtx, 5)) {
            if (sharedFresh) { s = shared; sharedFresh = false; have = true; }
            osMutexGive(&snapMtx);
        }
        if (have)
            logger.logRow(s.t_ms, s.out.appsADC, s.out.pedalPct, s.out.bpsADC,
                          s.out.brakePct, s.out.bppcCut, s.out.canStale,
                          s.out.invFault, s.out.torqueCmd, s.dti);
        osSleep(20);                        // 50 Hz, matching the command rate
    }
}

// Dump a completed single-shot capture as CSV. Runs in the telemetry task, so
// even though it is a long blocking print it cannot delay the control loop.
static void scopeDump() {
    Serial.println(F("# scope capture: 1000 samples @ 1 kHz"));
    Serial.println(F("i,apps_raw,apps_filt,bps_raw,bps_filt"));
    for (uint16_t i = 0; i < SCOPE_N; i++)
        Serial.printf("%u,%u,%u,%u,%u\n", i,
                      scopeBuf[i].rawA, scopeBuf[i].filtA,
                      scopeBuf[i].rawB, scopeBuf[i].filtB);
    Serial.println(F("# end"));
}

// Phase-3 comparison: run the shared input script through a FRESH VcuApp and
// print each output row as bare CSV (no kernel tag) so the miniOS and FreeRTOS
// logs diff to zero when the kernels are equivalent.
static void selfTestSink(int i, const VcuTestStep &in, const VcuOutputs &out) {
    Serial.printf("%d,%u,%u,%u,%u,%u,%.3f,%.3f,%d,%d,%d,%.3f,%d\n",
        i, in.apps, in.bps, in.fault, in.ageMs,
        out.appsADC, out.pedalPct, out.brakePct,
        out.bppcCut ? 1 : 0, out.canStale ? 1 : 0, out.invFault ? 1 : 0,
        out.torqueCmd, out.sendBrake ? 1 : 0);
}
static void runSelfTest() {
    Serial.println(F("# selftest begin "
        "step,apps,bps,fault,age,appsADC,pedalPct,brakePct,bppc,stale,inv,torque,sendBrake"));
    VcuApp t;                                   // own instance; live controller untouched
    vcuSelfTest(t, selfTestSink);
    Serial.println(F("# selftest end"));
}

// Soak diagnostic: per-task stack high-water. Prints the WORST-CASE stack use
// seen so far (total - free). If these numbers grow between the start and end
// of a long soak, a task is leaking stack. Also prints uptime and the CAN
// freshness so one line captures overall health.
static void stackDump() {
    Serial.printf("# stacks uptime_ms %lu\n", (unsigned long)millis());
    uint32_t total = osStackTotalWords();
    for (int i = 0; i < osTaskCount(); i++) {
        const char *nm = osTaskName(i);
        if (!nm) continue;
        uint32_t freeW = osStackFreeWords(i);
        Serial.printf("#   %-6s used %4lu / %lu words  (free %lu)\n",
                      nm, (unsigned long)(total - freeW), (unsigned long)total,
                      (unsigned long)freeW);
    }
}

// Console: single non-blocking char selects the output mode.
static void scopeConsole() {
    while (Serial.available()) {
        switch (Serial.read()) {
            case 'T': runSelfTest(); break;     // Phase-3 kernel-equivalence test
            case 'k': stackDump();   break;     // soak: stack high-water marks
            case 'c':                       // single-shot capture
                scopeIdx = 0; scopeReady = false; scopeArmed = true;
                Serial.println(F("[scope] capturing 1 s @ 1 kHz..."));
                break;
            case 'l':                       // live Serial-Plotter stream
                scopeLive = true;
                Serial.println(F("[scope] live (apps_raw apps_filt bps_raw bps_filt)"));
                break;
            case 'x':                       // stop live, back to telemetry
            case 't':
                scopeLive = false;
                Serial.println(F("[scope] telemetry"));
                break;
            case 'h':
                Serial.println(F("[scope] c=capture  l=live plot  x=stop  t=telemetry"));
                break;
            default: break;
        }
    }
}

// ===================== telemetryTask (prio 1) =====================
static void telemetryTask(void *) {
    // Timebase for the rate readouts. DWT_CYCCNT counts CORE cycles, so it is
    // only a valid clock while the core is actually running -- it halts on wfi.
    // That is exactly why miniOS's idle task spins instead of sleeping; see the
    // note on idleTask in miniOS.cpp. Do not reintroduce wfi without also
    // moving these measurements (and millis()-based timeouts) off this counter.
    ARM_DEMCR    |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
    const uint32_t cycPerMs = F_CPU_ACTUAL / 1000UL;

    uint32_t lastLoops = 0, lastMs = millis(),
             lastTick  = osTickCount(), lastCyc = ARM_DWT_CYCCNT;
    for (;;) {
        scopeConsole();                     // check for a mode command every pass

        // A finished capture takes priority: dump it, then carry on.
        if (scopeReady) { scopeDump(); scopeReady = false; lastCyc = ARM_DWT_CYCCNT; }

        // Live plot: stream the latest raw+filtered pair fast enough to see the
        // signal move. Tab-separated for the Arduino Serial Plotter.
        if (scopeLive) {
            Serial.printf("%u\t%u\t%u\t%u\n",
                          liveRawA, liveFiltA, liveRawB, liveFiltB);
            osSleep(20);                    // ~50 Hz
            continue;
        }

        uint32_t cyc    = ARM_DWT_CYCCNT;
        uint32_t realMs = (cyc - lastCyc) / cycPerMs;      // TRUE elapsed time
        if (realMs < TELEMETRY_MS) { osSleep(5); continue; }

        Snapshot s; bool have = false;
        if (osMutexTake(&snapMtx, 5)) { s = shared; have = true; osMutexGive(&snapMtx); }

        uint32_t now   = millis();
        uint32_t loops = ctrlLoops, tick = osTickCount();
        uint32_t hz    = ((loops - lastLoops) * 1000UL) / realMs;  // control loops/sec
        uint32_t tps   = ((tick  - lastTick)  * 1000UL) / realMs;  // kernel ticks/sec
        uint32_t mps   = ((now   - lastMs)    * 1000UL) / realMs;  // millis()/sec: 1000 = honest
        lastLoops = loops; lastTick = tick; lastMs = now; lastCyc = cyc;

        if (have)
            Serial.printf("[mini] APPS %u(%.0f%%) BPS %u(%.0f%%) BPPC %s stale %d inv %d "
                          "| trq %.1f Vin %.1f Flt %u | ctrl %lu tick %lu millis %lu\n",
                s.out.appsADC, s.out.pedalPct, s.out.bpsADC, s.out.brakePct,
                s.out.bppcCut ? "CUT" : "ok", s.out.canStale, s.out.invFault,
                s.out.torqueCmd, s.dti.Actual_InputVoltage,
                s.dti.Actual_FaultCode,
                (unsigned long)hz, (unsigned long)tps, (unsigned long)mps);

        // Periodic health line for soak testing: one line every ~10 s carrying
        // uptime + per-task stack high-water. A reboot is then unmissable (uptime
        // jumps back to near 0) and a stack leak shows as a task's number
        // climbing over the run -- both visible in a plain continuous capture,
        // with no dependence on when the port happens to be read.
        static uint32_t lastHealth = 0;
        if (now - lastHealth >= 10000) {
            lastHealth = now;
            uint32_t total = osStackTotalWords();
            Serial.printf("# health up %lu ms |", (unsigned long)now);
            for (int i = 0; i < osTaskCount(); i++) {
                const char *nm = osTaskName(i);
                if (nm) Serial.printf(" %s %lu", nm,
                                      (unsigned long)(total - osStackFreeWords(i)));
            }
            Serial.println();
        }
        osSleep(5);
    }
}

// ===================== watchdogTask (prio 5) =====================
static void watchdogTask(void *) {
    for (;;) {
        if (!osWatchdogHealthy())
            Serial.println("[mini] WATCHDOG: a task missed its check-in");
        osWatchdogSweep();
        osSleep(WATCHDOG_MS);
    }
}

// ===================== boot =====================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    pinMode(APPS_PIN, INPUT);
    pinMode(BPS_PIN, INPUT);
    analogReadResolution(10);
    analogReadAveraging(16);

    Can2.begin();
    Can2.setBaudRate(500000);
    logger.begin("minilog");

    osMutexInit(&dtiMtx);
    osMutexInit(&canMtx);
    osMutexInit(&snapMtx);

    osCreateTask(controlTask,   "CTRL",  7, nullptr);
    osCreateTask(canRxTask,     "CANRX", 6, nullptr);
    osCreateTask(watchdogTask,  "WDOG",  5, nullptr);
    osCreateTask(logTask,       "LOG",   2, nullptr);
    osCreateTask(telemetryTask, "TELEM", 1, nullptr);

    Serial.println("VCU on miniOS - starting scheduler");
    osStart(1000);                          // 1 ms tick; never returns
}

void loop() {}
