/*
 * miniOS - a small fixed-priority preemptive kernel for Teensy 4.1 (Cortex-M7F)
 * ============================================================================
 * PHASE 1 of the custom-OS trial. Everything is statically allocated: no malloc
 * after init, no heap fragmentation over a long run.
 *
 * What it has now (vs. the round-robin toy):
 *   - Fixed-priority preemptive scheduling  (highest READY task always runs)
 *   - Real blocking: osSleep() leaves the ready set and frees the CPU
 *   - Mutex with PRIORITY INHERITANCE (prevents unbounded priority inversion)
 *   - FPU context save/restore (S16-S31 + per-task EXC_RETURN) so float math
 *     survives preemption -- the VCU control path is float-heavy
 *   - Round-robin only among EQUAL priorities
 *
 * Phase 2 added below: stack canaries + high-water reporting, fault handlers
 * with named reporting, cooperative watchdog.
 *
 * Still missing vs. FreeRTOS: queues, counting semaphores, event groups,
 * software timers, MPU-backed stack guards, tickless idle.
 */
#pragma once
#include <Arduino.h>

#define OS_MAX_TASKS   8
// 6 KB per task. Raised from the 512-word trial value because the VCU tasks do
// Serial.printf (~300 words measured on the trial) and SdFat writes, which are
// stack-hungry. 8 x 1536 words = 48 KB, trivial against ~360 KB free.
#define OS_STACK_WORDS 1536
#define OS_MAX_PRIO    8            // 0 = lowest (idle), 7 = highest

typedef void (*os_task_fn)(void *);

enum OsState : uint8_t { OS_READY, OS_BLOCKED, OS_SLEEPING };

struct OsMutex;

struct OsTCB {
    uint32_t *sp;                   // MUST be first: the asm dereferences [tcb]
    uint32_t  wakeTick;             // when a sleeper/timeout becomes ready again
    OsMutex  *waitingOn;            // mutex this task is blocked on (or nullptr)
    // volatile: tasks spin on this while the tick ISR / another task changes it,
    // so the compiler must re-read it rather than cache it into a register.
    volatile OsState state;
    uint8_t   basePrio;             // priority the task was created with
    uint8_t   curPrio;              // current (may be boosted by inheritance)
    bool      used;
    bool      wdRegistered;         // participates in the cooperative watchdog
    volatile bool wdCheckedIn;      // has checked in this watchdog window
    const char *name;
};

struct OsMutex {
    OsTCB   *owner;                 // nullptr = free
    uint8_t  ownerSavedPrio;        // owner's priority before any boost
    bool     initialised;
};

// ---- kernel API ----
bool     osCreateTask(os_task_fn fn, const char *name, uint8_t prio, void *arg);
void     osStart(uint32_t tickMicros);   // never returns
void     osSleep(uint32_t ms);           // block this task, free the CPU (RELATIVE)
void     osYield();                      // give up the rest of this slice
uint32_t osTickCount();

/*
 * Drift-free periodic sleep -- the equivalent of FreeRTOS vTaskDelayUntil().
 *
 * osSleep(1) means "1 ms from NOW", so a periodic loop's true period becomes
 * 1 ms + however long the work took, and that error repeats every cycle. On a
 * control loop it shows up as running measurably slower than intended.
 *
 * osSleepUntil() advances an ABSOLUTE deadline instead, so the period stays
 * exact regardless of execution time:
 *
 *   uint32_t next = osTickCount();
 *   for (;;) { work(); osSleepUntil(&next, 1); }
 */
void     osSleepUntil(uint32_t *lastWake, uint32_t periodMs);

// Last tick rate measured by osTickCalibrate(), in Hz. 0 until that has run.
uint32_t osTickHz();

/*
 * Measure the real tick rate and rescale the PIT reload to hit wantHz exactly.
 * Call ONCE from a task after startup. IntervalTimer's requested period lands
 * within ~2% here, which is too loose for a 1 kHz control loop that the command
 * rate, slew limit and every timeout are divided down from.
 *
 * MUST be called from a running task, not from osStart(): it measures against
 * ARM_DWT_CYCCNT, which counts CORE cycles and therefore only advances while the
 * core is awake. It spins to keep the core busy for its measurement window.
 * Returns the achieved rate (0 on failure).
 */
uint32_t osTickCalibrate(uint32_t wantHz);

void     osMutexInit(OsMutex *m);
bool     osMutexTake(OsMutex *m, uint32_t timeoutMs);
void     osMutexGive(OsMutex *m);

// ================= Phase 2: diagnostics & survivability =================

// Stack instrumentation. Stacks are filled with a pattern at creation; the
// lowest OS_CANARY_WORDS are a guard band checked on every context switch.
#define OS_FILL_WORD    0xA5A5A5A5u
#define OS_CANARY_WORDS 8            // 32-byte guard at the bottom of each stack

int         osTaskCount();
const char *osTaskName(int idx);
uint32_t    osStackFreeWords(int idx);   // high-water: untouched words remaining
uint32_t    osStackTotalWords();

// Cooperative watchdog: every monitored task must check in each window, or the
// dog starves. A hung task therefore cannot be masked by healthy ones.
void osWatchdogRegister();               // call once from inside a task
void osTaskCheckIn();                    // call periodically from that task
bool osWatchdogHealthy();                // true if ALL registered tasks checked in
void osWatchdogSweep();                  // clear the check-ins; start a new window

// Fault reporting: captured by the HardFault/MemManage/BusFault/UsageFault
// handlers. faulted==true means the kernel trapped a CPU fault.
struct OsFaultInfo {
    bool     faulted;
    uint32_t pc, lr, psr, cfsr, hfsr, faultAddr;
    const char *taskName;
    const char *kind;
};
const OsFaultInfo *osLastFault();
