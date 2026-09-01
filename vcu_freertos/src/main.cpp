/*
 * Formula Manipal VCU - FreeRTOS ADAPTER  -  Teensy 4.1
 * =====================================================
 * NO control logic here. All of it is in ../shared_libs/VcuApp, shared
 * byte-for-byte with the miniOS build.
 *
 * Task table
 *   ControlTask    prio 8  1 kHz   sample -> (50 Hz) control -> CAN TX
 *   CanRxTask      prio 7  event   drain CAN -> DTI550_decode() under mutex
 *   WatchdogTask   prio 6  10 Hz   cooperative check-in monitor
 *   LogTask        prio 2  queue   SD writes (blocking I/O isolated here)
 *   TelemetryTask  prio 1  10 Hz   Serial (lowest; can never delay safety)
 *
 * Rules that keep the safety path safe:
 *   - ControlTask never blocks: mutex takes use short timeouts and fall back to
 *     the last known-good snapshot.
 *   - The log queue is non-blocking: drops rows under backpressure rather than
 *     stalling the control loop.
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "arduino_freertos.h"
#include "semphr.h"
#include "queue.h"
using namespace arduino;          // the header #undefs Arduino names; re-scope them

#include "DTI550.h"
#include "VcuApp.h"
#include "VcuSelfTest.h"
#include "DataLogger.h"

void DTI550_decode(const CAN_message_t &msg);   // generated .cpp defines it

// ---------------- Pins / rates ----------------
#define APPS_PIN A2
#define BPS_PIN  A1
static const uint32_t CTRL_PERIOD_MS = 1;     // 1 kHz sampling
static const uint32_t CMD_DIVIDER    = 20;    // -> 50 Hz command
static const uint32_t TELEMETRY_MS   = 100;
static const uint32_t WATCHDOG_MS    = 100;

// ---------------- DTI command frames ----------------
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
static VcuApp     app;                     // <-- the shared control logic

// Measured control-loop rate. Should read ~1000 Hz; anything lower means the
// control task is missing its deadline.
static volatile uint32_t ctrlLoops = 0;
static DataLogger logger;

static SemaphoreHandle_t dtiMutex, canMutex, snapMutex;
static QueueHandle_t     logQueue;
static volatile uint32_t dtiLastUpdateMs = 0;

struct Snapshot { VcuOutputs out; DTI550_Data dti; uint32_t t_ms; };
static Snapshot telemSnap;

static const uint32_t ALIVE_CONTROL = 1 << 0, ALIVE_CANRX = 1 << 1;
static const uint32_t ALIVE_ALL = ALIVE_CONTROL | ALIVE_CANRX;
static volatile uint32_t aliveFlags = 0;

// ===================== CanRxTask (prio 7) =====================
static void CanRxTask(void *) {
    CAN_message_t rx;
    for (;;) {
        bool got = false;
        if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            got = Can2.read(rx);
            xSemaphoreGive(canMutex);
        }
        if (got) {
            if (xSemaphoreTake(dtiMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                DTI550_decode(rx);
                xSemaphoreGive(dtiMutex);
            }
            if (isDtiFeedback(rx.id)) dtiLastUpdateMs = millis();
        } else {
            vTaskDelay(1);
        }
        aliveFlags |= ALIVE_CANRX;
    }
}

// ===================== ControlTask (prio 8) =====================
static void ControlTask(void *) {
    uint32_t    div = 0;
    DTI550_Data dtiCopy{};                 // last known-good feedback
    TickType_t  next = xTaskGetTickCount();

    for (;;) {
        // --- 1 kHz: filtering only ---
        app.sample(analogRead(APPS_PIN), analogRead(BPS_PIN));

        // --- 50 Hz: the control decision ---
        if (++div >= CMD_DIVIDER) {
            div = 0;

            // short timeout: never stall the safety path on a contended mutex
            if (xSemaphoreTake(dtiMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                dtiCopy = dti550;
                xSemaphoreGive(dtiMutex);
            }

            VcuInputs  in{ millis(), dtiLastUpdateMs, dtiCopy };
            VcuOutputs out{};
            app.control(in, out);          // <-- identical call in both kernels

            if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                Can2.write(dtiDriveEnable(true));
                Can2.write(out.sendBrake ? dtiScaled(PID_REL_BRAKE, out.brakePct)
                                         : dtiScaled(PID_REL_CURRENT, out.torqueCmd));
                xSemaphoreGive(canMutex);
            }

            Snapshot s{ out, dtiCopy, millis() };
            xQueueSend(logQueue, &s, 0);   // 0 timeout: drop, never stall
            if (xSemaphoreTake(snapMutex, 0) == pdTRUE) {
                telemSnap = s;
                xSemaphoreGive(snapMutex);
            }
        }

        ctrlLoops++;                       // for the measured-rate readout
        aliveFlags |= ALIVE_CONTROL;
        vTaskDelayUntil(&next, pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

// ===================== LogTask (prio 2) =====================
static void LogTask(void *) {
    Snapshot s;
    for (;;)
        if (xQueueReceive(logQueue, &s, portMAX_DELAY) == pdTRUE)
            logger.logRow(s.t_ms, s.out.appsADC, s.out.pedalPct, s.out.bpsADC,
                          s.out.brakePct, s.out.bppcCut, s.out.canStale,
                          s.out.invFault, s.out.torqueCmd, s.dti);
}

// Phase-3 comparison: identical to the miniOS build. Bare CSV, no kernel tag,
// same field order and formatting, so the two logs diff to zero when the
// kernels are equivalent.
static void selfTestSink(int i, const VcuTestStep &in, const VcuOutputs &out) {
    Serial.printf("%d,%u,%u,%u,%u,%u,%.3f,%.3f,%d,%d,%d,%.3f,%d\n",
        i, in.apps, in.bps, in.fault, in.ageMs,
        out.appsADC, out.pedalPct, out.brakePct,
        out.bppcCut ? 1 : 0, out.canStale ? 1 : 0, out.invFault ? 1 : 0,
        out.torqueCmd, out.sendBrake ? 1 : 0);
}
static void runSelfTest() {
    Serial.println("# selftest begin "
        "step,apps,bps,fault,age,appsADC,pedalPct,brakePct,bppc,stale,inv,torque,sendBrake");
    VcuApp t;
    vcuSelfTest(t, selfTestSink);
    Serial.println("# selftest end");
}

// ===================== TelemetryTask (prio 1) =====================
static void TelemetryTask(void *) {
    TickType_t next = xTaskGetTickCount();
    uint32_t   lastLoops = 0, lastMs = millis();
    for (;;) {
        if (Serial.available() && Serial.read() == 'T') runSelfTest();

        Snapshot s;
        if (xSemaphoreTake(snapMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            s = telemSnap;
            xSemaphoreGive(snapMutex);

            // measured control-loop rate (target 1000 Hz)
            uint32_t now = millis(), loops = ctrlLoops;
            uint32_t hz  = (now > lastMs) ? ((loops - lastLoops) * 1000UL) / (now - lastMs) : 0;
            lastLoops = loops; lastMs = now;

            Serial.printf("[RTOS] APPS %u(%.0f%%) BPS %u(%.0f%%) BPPC %s stale %d inv %d "
                          "| trq %.1f ERPM %.0f Vin %.1f Fault %u | ctrl %lu Hz\n",
                s.out.appsADC, s.out.pedalPct, s.out.bpsADC, s.out.brakePct,
                s.out.bppcCut ? "CUT" : "ok", s.out.canStale, s.out.invFault,
                s.out.torqueCmd, s.dti.Actual_ERPM, s.dti.Actual_InputVoltage,
                s.dti.Actual_FaultCode, (unsigned long)hz);
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(TELEMETRY_MS));
    }
}

// ===================== WatchdogTask (prio 6) =====================
static void WatchdogTask(void *) {
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        if ((aliveFlags & ALIVE_ALL) == ALIVE_ALL) {
            // wdt.feed();   <-- enable together with the safe-start guard:
            //                   a reset must not bring the car up into torque.
            aliveFlags = 0;
        } else {
            Serial.println("[RTOS] WATCHDOG: a task missed its check-in");
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(WATCHDOG_MS));
    }
}

// ===================== boot =====================
void setup() {
    Serial.begin(115200);
    pinMode(APPS_PIN, INPUT);
    pinMode(BPS_PIN, INPUT);
    analogReadResolution(10);
    analogReadAveraging(16);

    Can2.begin();
    Can2.setBaudRate(500000);
    logger.begin("rtoslog");

    dtiMutex  = xSemaphoreCreateMutex();
    canMutex  = xSemaphoreCreateMutex();
    snapMutex = xSemaphoreCreateMutex();
    logQueue  = xQueueCreate(32, sizeof(Snapshot));
    if (!dtiMutex || !canMutex || !snapMutex || !logQueue) {
        Serial.println("FATAL: RTOS object creation failed");
        while (1) {}
    }

    xTaskCreate(ControlTask,   "CTRL",  2048, NULL, 8, NULL);
    xTaskCreate(CanRxTask,     "CANRX", 1536, NULL, 7, NULL);
    xTaskCreate(WatchdogTask,  "WDOG",  1024, NULL, 6, NULL);
    xTaskCreate(LogTask,       "LOG",   3072, NULL, 2, NULL);
    xTaskCreate(TelemetryTask, "TELEM", 2048, NULL, 1, NULL);

    Serial.println("VCU on FreeRTOS - starting scheduler");
    vTaskStartScheduler();
}

void loop() {}
