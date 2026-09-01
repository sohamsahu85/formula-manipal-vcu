#include "miniOS.h"

// ---- raw system registers (don't depend on core macro spellings) ----
#define MEM32(a)       (*(volatile uint32_t *)(a))
#define ICSR           MEM32(0xE000ED04)   // Interrupt Control/State
#define SHPR3          MEM32(0xE000ED20)   // System Handler Priority 3 (PendSV/SysTick)
#define FPCCR          MEM32(0xE000EF34)   // FP Context Control (ASPEN/LSPEN)
#define ICSR_PENDSVSET (1u << 28)

// ============================ kernel state ============================
static OsTCB    tasks[OS_MAX_TASKS];
static uint32_t stacks[OS_MAX_TASKS][OS_STACK_WORDS] __attribute__((aligned(8)));
static volatile uint32_t osTicks = 0;
static bool     schedulerRunning = false;

// The PendSV assembly reads/writes these two. C linkage so `ldr r0,=osCurrent`
// resolves. osCurrent == nullptr means "first ever switch, nothing to save".
extern "C" {
    OsTCB *osCurrent = nullptr;
    OsTCB *osNext    = nullptr;
}

static IntervalTimer tickTimer;

// ---- critical section (save/restore PRIMASK so it nests safely) ----
// The Teensy 4 core doesn't pull in the CMSIS intrinsics, so read PRIMASK directly.
static inline uint32_t critEnter() {
    uint32_t pm;
    __asm volatile("mrs %0, primask" : "=r"(pm));
    __asm volatile("cpsid i" ::: "memory");
    return pm;
}
static inline void critExit(uint32_t pm) {
    if (!pm) __asm volatile("cpsie i" ::: "memory");
}

uint32_t osTickCount() { return osTicks; }

// forward decls (Phase 2 diagnostics, defined below but used by the tick)
static bool osCanaryIntact(int i);
static void osStackOverflowTrap(int i);

// ============================ scheduler ============================
/*
 * The whole scheduling policy, in one function: pick the highest-priority READY
 * task; among equals, continue past the current one so equal priorities
 * round-robin. If the winner isn't the running task, pend a context switch.
 *
 * Called from the tick, from osSleep(), and from the mutex calls -- i.e. at
 * every point where the set of runnable tasks can change.
 */
static void osSchedule() {
    int best = -1;
    int bestPrio = -1;

    // start scanning just after the current task so equal priorities rotate
    int startIdx = 0;
    if (osCurrent) {
        for (int i = 0; i < OS_MAX_TASKS; i++)
            if (&tasks[i] == osCurrent) { startIdx = i + 1; break; }
    }

    for (int n = 0; n < OS_MAX_TASKS; n++) {
        int i = (startIdx + n) % OS_MAX_TASKS;
        if (!tasks[i].used || tasks[i].state != OS_READY) continue;
        if ((int)tasks[i].curPrio > bestPrio) { bestPrio = tasks[i].curPrio; best = i; }
    }

    if (best < 0) return;                       // nothing ready (idle covers this)
    osNext = &tasks[best];
    if (osNext != osCurrent) ICSR = ICSR_PENDSVSET;
}

// ---- tick: age the sleepers, then re-decide who runs ----
static int osPitCh = -1;                 // PIT channel backing the kernel tick

static void osTickISR() {
    // Clear the PIT interrupt flag FIRST. If it stays set, the interrupt is
    // still asserted and re-fires on every return to thread mode -- i.e. once
    // per context switch -- inflating the tick rate by roughly the task count.
    if (osPitCh >= 0) IMXRT_PIT_CHANNELS[osPitCh].TFLG = 1;

    osTicks++;

    // The timer is started BEFORE the scheduler is armed (see osStart), so a few
    // ticks can land while there is no current task. Count them and leave.
    if (!schedulerRunning) return;

    // Heartbeat: LED toggles at 2 Hz straight from the tick. If this blinks the
    // kernel tick is alive even when no task output appears -- a fast way to
    // tell "tick dead" from "tasks stuck".
    if ((osTicks % 250) == 0) digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));

    // Stack guard: catch an overflow while we still know WHOSE it was. Without
    // this, an overflow silently eats the neighbouring task's stack and the
    // crash appears somewhere unrelated, minutes later.
    for (int i = 0; i < OS_MAX_TASKS; i++)
        if (tasks[i].used && !osCanaryIntact(i)) osStackOverflowTrap(i);

    for (int i = 0; i < OS_MAX_TASKS; i++) {
        OsTCB *t = &tasks[i];
        if (!t->used) continue;
        // A sleeper wakes on time; a mutex waiter wakes on timeout (wakeTick != 0)
        if ((t->state == OS_SLEEPING || (t->state == OS_BLOCKED && t->wakeTick)) &&
            (int32_t)(osTicks - t->wakeTick) >= 0) {
            t->state     = OS_READY;
            t->waitingOn = nullptr;             // timed out waiting for the mutex
            t->wakeTick  = 0;
        }
    }
    osSchedule();
}

// ============================ task creation ============================
static void osTaskExit() { for (;;) {} }        // a task must never return

bool osCreateTask(os_task_fn fn, const char *name, uint8_t prio, void *arg) {
    if (prio >= OS_MAX_PRIO) return false;

    for (int i = 0; i < OS_MAX_TASKS; i++) {
        if (tasks[i].used) continue;

        // Fill the WHOLE stack with a pattern first: it seeds the bottom canary
        // words AND lets us measure the high-water mark later by counting how
        // many words are still untouched.
        for (int w = 0; w < OS_STACK_WORDS; w++) stacks[i][w] = OS_FILL_WORD;

        uint32_t *sp = &stacks[i][OS_STACK_WORDS];   // stacks grow DOWN

        // --- exception frame the hardware will pop on exception return ---
        *(--sp) = 0x01000000;              // xPSR (Thumb bit -- mandatory)
        *(--sp) = (uint32_t)fn;            // PC
        *(--sp) = (uint32_t)osTaskExit;    // LR
        *(--sp) = 0;                       // R12
        *(--sp) = 0;                       // R3
        *(--sp) = 0;                       // R2
        *(--sp) = 0;                       // R1
        *(--sp) = (uint32_t)arg;           // R0  -> the task's argument

        // --- software-saved block; memory order low->high must be r4..r11,LR ---
        *(--sp) = 0xFFFFFFFD;              // EXC_RETURN: Thread mode, PSP, NO FP yet
        for (int r = 0; r < 8; r++) *(--sp) = 0;   // R11..R4

        // field-by-field (the volatile member blocks aggregate copy-assignment)
        tasks[i].sp           = sp;
        tasks[i].wakeTick     = 0;
        tasks[i].waitingOn    = nullptr;
        tasks[i].state        = OS_READY;
        tasks[i].basePrio     = prio;
        tasks[i].curPrio      = prio;
        tasks[i].name         = name;
        tasks[i].wdRegistered = false;
        tasks[i].wdCheckedIn  = false;
        tasks[i].used         = true;   // last: slot goes live only when complete
        return true;
    }
    return false;
}

// ============================ blocking ============================
void osSleep(uint32_t ms) {
    if (!schedulerRunning) { delay(ms); return; }
    uint32_t pm = critEnter();
    osCurrent->wakeTick = osTicks + (ms ? ms : 1);   // tick == 1 ms in this build
    osCurrent->state    = OS_SLEEPING;
    osSchedule();
    critExit(pm);
    while (osCurrent->state == OS_SLEEPING) { }      // switch happens on PendSV
}

void osYield() {
    uint32_t pm = critEnter();
    osSchedule();
    critExit(pm);
}

/*
 * Drift-free periodic sleep -- the FreeRTOS vTaskDelayUntil() equivalent.
 * Advances an ABSOLUTE deadline instead of sleeping a relative amount, so the
 * period does not stretch by however long the task's work took.
 */
void osSleepUntil(uint32_t *lastWake, uint32_t periodMs) {
    if (!schedulerRunning) { delay(periodMs); return; }
    if (periodMs == 0) periodMs = 1;

    uint32_t pm = critEnter();
    uint32_t wake = *lastWake + periodMs;

    // Already past the deadline? The last pass overran its period. Resync
    // rather than trying to catch up, so one slow cycle can't cascade into a
    // burst of back-to-back runs.
    if ((int32_t)(osTicks - wake) >= 0) {
        *lastWake = osTicks;
        critExit(pm);
        osYield();                       // still give equal/higher tasks a turn
        return;
    }

    *lastWake           = wake;
    osCurrent->wakeTick = wake;
    osCurrent->state    = OS_SLEEPING;
    osSchedule();
    critExit(pm);
    while (osCurrent->state == OS_SLEEPING) { }
}

// ============================ mutex + priority inheritance ============================
/*
 * WHY THIS EXISTS (the classic RTOS bug):
 *   low holds the mutex -> high blocks on it -> medium preempts low
 *   => high waits behind medium, unbounded. That's priority inversion.
 *
 * The fix: while a high-priority task waits on a mutex, temporarily raise the
 * OWNER to the waiter's priority so medium can't jump ahead. Restore on give.
 * A binary semaphore cannot do this -- it has no owner. That is precisely the
 * difference between a mutex and a semaphore.
 */
void osMutexInit(OsMutex *m) { m->owner = nullptr; m->ownerSavedPrio = 0; m->initialised = true; }

bool osMutexTake(OsMutex *m, uint32_t timeoutMs) {
    uint32_t pm = critEnter();

    if (m->owner == nullptr) {                       // free -> take it
        m->owner = osCurrent;
        m->ownerSavedPrio = osCurrent->curPrio;
        critExit(pm);
        return true;
    }
    if (m->owner == osCurrent) { critExit(pm); return true; }   // already ours

    // --- PRIORITY INHERITANCE: boost the owner to our priority ---
    if (m->owner->curPrio < osCurrent->curPrio) m->owner->curPrio = osCurrent->curPrio;

    osCurrent->waitingOn = m;
    osCurrent->state     = OS_BLOCKED;
    osCurrent->wakeTick  = timeoutMs ? (osTicks + timeoutMs) : 0;   // 0 = wait forever
    osSchedule();
    critExit(pm);

    while (osCurrent->state == OS_BLOCKED) { }       // blocked: CPU goes elsewhere

    pm = critEnter();
    bool got = (m->owner == nullptr);
    if (got) { m->owner = osCurrent; m->ownerSavedPrio = osCurrent->curPrio; }
    osCurrent->waitingOn = nullptr;
    critExit(pm);
    return got;
}

void osMutexGive(OsMutex *m) {
    uint32_t pm = critEnter();
    if (m->owner != osCurrent) { critExit(pm); return; }

    osCurrent->curPrio = m->owner->basePrio;         // drop any inherited boost
    m->owner = nullptr;

    // wake the highest-priority waiter
    OsTCB *best = nullptr;
    for (int i = 0; i < OS_MAX_TASKS; i++) {
        OsTCB *t = &tasks[i];
        if (t->used && t->state == OS_BLOCKED && t->waitingOn == m)
            if (!best || t->curPrio > best->curPrio) best = t;
    }
    if (best) { best->state = OS_READY; best->wakeTick = 0; }

    osSchedule();
    critExit(pm);
}

// ==================== Phase 2: stack instrumentation ====================
int         osTaskCount()        { return OS_MAX_TASKS; }
const char *osTaskName(int i)    { return (i >= 0 && i < OS_MAX_TASKS && tasks[i].used) ? tasks[i].name : nullptr; }
uint32_t    osStackTotalWords()  { return OS_STACK_WORDS; }

// High-water mark: count untouched fill words from the bottom up. Small number
// = close to overflow. This is how you right-size stacks by measurement.
uint32_t osStackFreeWords(int i) {
    if (i < 0 || i >= OS_MAX_TASKS || !tasks[i].used) return 0;
    uint32_t n = 0;
    while (n < OS_STACK_WORDS && stacks[i][n] == OS_FILL_WORD) n++;
    return n;
}

// Canary check: the lowest OS_CANARY_WORDS must still hold the fill pattern.
static bool osCanaryIntact(int i) {
    for (int w = 0; w < OS_CANARY_WORDS; w++)
        if (stacks[i][w] != OS_FILL_WORD) return false;
    return true;
}

// Overflow trap: report which task blew its stack, then park. Named so the
// failure is obvious instead of showing up as a random fault elsewhere.
static void osStackOverflowTrap(int i) {
    __asm volatile("cpsid i");
    Serial.printf("\n*** STACK OVERFLOW in task '%s' (slot %d)\n",
                  tasks[i].name ? tasks[i].name : "?", i);
    Serial.println("    the bottom guard words were overwritten -- increase OS_STACK_WORDS");
    Serial.flush();
    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) {                                   // fast blink = stack overflow
        digitalWriteFast(LED_BUILTIN, 1); delayMicroseconds(60000);
        digitalWriteFast(LED_BUILTIN, 0); delayMicroseconds(60000);
    }
}

// ======================= Phase 2: cooperative watchdog =======================
void osWatchdogRegister() { if (osCurrent) { osCurrent->wdRegistered = true; osCurrent->wdCheckedIn = true; } }
void osTaskCheckIn()      { if (osCurrent) osCurrent->wdCheckedIn = true; }

bool osWatchdogHealthy() {
    for (int i = 0; i < OS_MAX_TASKS; i++)
        if (tasks[i].used && tasks[i].wdRegistered && !tasks[i].wdCheckedIn) return false;
    return true;
}
void osWatchdogSweep() {
    for (int i = 0; i < OS_MAX_TASKS; i++)
        if (tasks[i].used && tasks[i].wdRegistered) tasks[i].wdCheckedIn = false;
}

// ========================= Phase 2: fault handlers =========================
#define CFSR  MEM32(0xE000ED28)   // Configurable Fault Status
#define HFSR  MEM32(0xE000ED2C)   // HardFault Status
#define MMFAR MEM32(0xE000ED34)   // MemManage Fault Address
#define BFAR  MEM32(0xE000ED38)   // BusFault Address
#define SHCSR MEM32(0xE000ED24)   // System Handler Control and State

static OsFaultInfo lastFault;
const OsFaultInfo *osLastFault() { return &lastFault; }

/*
 * Without this, a bug in the kernel or a task is a SILENT hang -- the single
 * worst thing when bringing up your own OS. Here we decode which fault fired,
 * from which task, and at what PC, then park with a visible SOS blink.
 */
extern "C" void osFaultReport(uint32_t *frame, uint32_t excReturn) {
    lastFault.faulted   = true;
    lastFault.pc        = frame[6];         // stacked PC = the faulting instruction
    lastFault.lr        = frame[5];
    lastFault.psr       = frame[7];
    lastFault.cfsr      = CFSR;
    lastFault.hfsr      = HFSR;
    lastFault.taskName  = (osCurrent && osCurrent->name) ? osCurrent->name : "?";

    if (CFSR & 0x0080)      { lastFault.kind = "MemManage"; lastFault.faultAddr = MMFAR; }
    else if (CFSR & 0x8000) { lastFault.kind = "BusFault";  lastFault.faultAddr = BFAR;  }
    else if (CFSR & 0x00FF) { lastFault.kind = "MemManage"; lastFault.faultAddr = 0;     }
    else if (CFSR & 0xFFFF0000u) { lastFault.kind = "UsageFault"; lastFault.faultAddr = 0; }
    else                    { lastFault.kind = "HardFault"; lastFault.faultAddr = 0;     }

    Serial.printf("\n*** %s in task '%s'\n", lastFault.kind, lastFault.taskName);
    Serial.printf("    PC=0x%08lX LR=0x%08lX PSR=0x%08lX\n",
                  (unsigned long)lastFault.pc, (unsigned long)lastFault.lr,
                  (unsigned long)lastFault.psr);
    Serial.printf("    CFSR=0x%08lX HFSR=0x%08lX ADDR=0x%08lX EXC_RET=0x%08lX\n",
                  (unsigned long)lastFault.cfsr, (unsigned long)lastFault.hfsr,
                  (unsigned long)lastFault.faultAddr, (unsigned long)excReturn);
    Serial.flush();

    pinMode(LED_BUILTIN, OUTPUT);
    for (;;) {                              // SOS: visibly "the kernel died here"
        for (int k = 0; k < 3; k++) { digitalWriteFast(LED_BUILTIN, 1); delayMicroseconds(120000);
                                      digitalWriteFast(LED_BUILTIN, 0); delayMicroseconds(120000); }
        delayMicroseconds(400000);
    }
}

// Naked trampoline: pick MSP or PSP (EXC_RETURN bit 2) and hand the frame over.
extern "C" __attribute__((naked)) void osFaultTrampoline(void) {
    __asm volatile(
        "  tst  lr, #4        \n"     // bit2: 0 = was using MSP, 1 = PSP
        "  ite  eq            \n"
        "  mrseq r0, msp      \n"
        "  mrsne r0, psp      \n"
        "  mov  r1, lr        \n"
        "  b    osFaultReport \n"
    );
}

// ============================ context switch ============================
/*
 * The heart of the kernel. On entry the hardware has already stacked
 * R0-R3, R12, LR, PC, xPSR (and S0-S15/FPSCR if the task used the FPU).
 * We save the rest, swap PSP, and restore the incoming task's.
 *
 * FPU handling (the part the round-robin toy skipped):
 *   EXC_RETURN bit 4 == 0  =>  this task has FP context stacked.
 *   So we conditionally save/restore S16-S31, and we store EXC_RETURN itself
 *   per-task -- because task A may be using the FPU while task B isn't.
 *   Get this wrong and floats silently corrupt across a preemption.
 */
extern "C" __attribute__((naked)) void PendSV_Handler(void) {
    __asm volatile(
        "  cpsid i                      \n"
        "  ldr   r0, =osCurrent         \n"
        "  ldr   r1, [r0]               \n"
        "  cbz   r1, 1f                 \n"   // first switch: nothing to save
        "  mrs   r2, psp                \n"
        "  tst   lr, #0x10              \n"   // bit4 clear => FP context active
        "  it    eq                     \n"
        "  vstmdbeq r2!, {s16-s31}      \n"   // save callee-saved FP regs
        "  stmdb r2!, {r4-r11, lr}      \n"   // save core regs + EXC_RETURN
        "  str   r2, [r1]               \n"   // osCurrent->sp = psp
        "1:                             \n"
        "  ldr   r0, =osNext            \n"
        "  ldr   r1, [r0]               \n"
        "  ldr   r2, [r1]               \n"   // r2 = osNext->sp
        "  ldmia r2!, {r4-r11, lr}      \n"   // restore core regs + EXC_RETURN
        "  tst   lr, #0x10              \n"
        "  it    eq                     \n"
        "  vldmiaeq r2!, {s16-s31}      \n"   // restore FP regs if this task has them
        "  msr   psp, r2                \n"
        "  ldr   r0, =osCurrent         \n"
        "  str   r1, [r0]               \n"   // osCurrent = osNext
        "  cpsie i                      \n"
        "  bx    lr                     \n"   // hardware pops the rest -> task runs
    );
}

// ============================ start ============================
/*
 * Idle SPINS -- deliberately -- rather than using "wfi".
 *
 * Measured on this board: bare wfi halts the core clock, which stops both
 * DWT_CYCCNT and SysTick. Since millis() is SysTick-derived and the CPU is idle
 * ~84% of the time at 1 kHz, millis() ran ~6x slow in real terms while the PIT
 * (a peripheral) kept correct time. Everything denominated in millis() -- most
 * importantly the CAN-staleness timeout in VcuApp -- silently stretched by that
 * factor. A 100 ms stale-CAN cutout would really have been ~600 ms.
 *
 * Spinning costs power we do not care about on a car, and buys a millis() that
 * matches real time. If low-power idle is ever wanted, the fix is to stop
 * deriving timeouts from millis() and use osTickCount() instead.
 */
static void idleTask(void *) { for (;;) { __asm volatile("nop"); } }

// The Teensy 4 core builds its vector table in RAM and does NOT use the stock
// CMSIS symbol names, so simply defining PendSV_Handler does not hook anything.
// Install it into the RAM vector table by hand. (Exception 14 = PendSV.)
// _VectorsRam itself is already declared by the core's imxrt.h.
extern "C" void PendSV_Handler(void);
extern "C" void osFaultTrampoline(void);

// ==================== tick calibration ====================

static uint32_t osTickHzActual = 0;
uint32_t osTickHz() { return osTickHzActual; }

/*
 * Runtime tick trim -- call ONCE from a task after startup.
 *
 * IntervalTimer's requested period lands within ~2% here, which is fine for
 * most things but not for a 1 kHz control loop that everything else is divided
 * down from. Measure the tick against the DWT cycle counter and rescale LDVAL
 * to hit the target exactly.
 *
 * IMPORTANT: this must run with the CPU busy. DWT_CYCCNT counts CORE cycles and
 * stops whenever the core is halted, so any measurement taken while the system
 * idles reads high. The spin loop below keeps the core awake for its window.
 */
uint32_t osTickCalibrate(uint32_t wantHz) {
    // locate the running PIT channel
    int ch = -1;
    for (int i = 0; i < 4; i++)
        if (IMXRT_PIT_CHANNELS[i].TCTRL & 1) { ch = i; break; }
    if (ch < 0) return 0;

    ARM_DEMCR    |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    // --- measure the tick rate we currently have ---
    uint32_t t0 = osTicks, c0 = ARM_DWT_CYCCNT;
    const uint32_t window = F_CPU_ACTUAL / 10;              // ~100 ms
    while ((ARM_DWT_CYCCNT - c0) < window) { }
    uint32_t dticks = osTicks - t0, dcyc = ARM_DWT_CYCCNT - c0;
    if (!dticks || !dcyc) return 0;

    uint32_t haveHz = (uint32_t)(((uint64_t)dticks * F_CPU_ACTUAL) / dcyc);
    if (!haveHz) return 0;

    // --- rescale the reload for the target rate ---
    uint32_t ld    = IMXRT_PIT_CHANNELS[ch].LDVAL;
    uint64_t newLd = ((uint64_t)(ld + 1) * haveHz) / wantHz;
    if (newLd < 2)          newLd = 2;
    if (newLd > 0xFFFFFFFE) newLd = 0xFFFFFFFE;

    // LDVAL only takes effect on the next reload, so stop and restart the
    // channel to apply it immediately.
    uint32_t tctrl = IMXRT_PIT_CHANNELS[ch].TCTRL;
    IMXRT_PIT_CHANNELS[ch].TCTRL = 0;
    IMXRT_PIT_CHANNELS[ch].LDVAL = (uint32_t)(newLd - 1);
    IMXRT_PIT_CHANNELS[ch].TCTRL = tctrl;

    // --- verify what we actually got ---
    t0 = osTicks; c0 = ARM_DWT_CYCCNT;
    while ((ARM_DWT_CYCCNT - c0) < window) { }
    dticks = osTicks - t0; dcyc = ARM_DWT_CYCCNT - c0;
    osTickHzActual = dcyc ? (uint32_t)(((uint64_t)dticks * F_CPU_ACTUAL) / dcyc) : 0;

    Serial.printf("[miniOS] tick calibrate: had %lu Hz (LDVAL %lu) -> now %lu Hz (LDVAL %lu)\n",
                  (unsigned long)haveHz, (unsigned long)ld,
                  (unsigned long)osTickHzActual, (unsigned long)(newLd - 1));
    return osTickHzActual;
}

void osStart(uint32_t tickMicros) {
    pinMode(LED_BUILTIN, OUTPUT);                    // for the tick heartbeat
    osCreateTask(idleTask, "IDLE", 0, nullptr);      // prio 0: runs when nothing else can

    _VectorsRam[14] = PendSV_Handler;                // hook the context switch

    // Hook the fault vectors so a crash self-reports instead of hanging silently.
    _VectorsRam[3] = osFaultTrampoline;              // HardFault
    _VectorsRam[4] = osFaultTrampoline;              // MemManage
    _VectorsRam[5] = osFaultTrampoline;              // BusFault
    _VectorsRam[6] = osFaultTrampoline;              // UsageFault
    SHCSR |= (1u << 18) | (1u << 17) | (1u << 16);   // enable Usage/Bus/MemManage
                                                     // (else they escalate to HardFault)

    SHPR3 = (SHPR3 & 0xFF00FFFF) | (0xFFu << 16);    // PendSV = lowest priority
    FPCCR |= (1u << 31) | (1u << 30);                // ASPEN|LSPEN: auto + lazy FP stacking

    // Start the tick with interrupts ENABLED, in the same environment the
    // calibration probe ran in. Starting it inside a cpsid-i section produced a
    // tick that ignored its requested period entirely (~6.6 kHz regardless of
    // the value passed), while the identical call outside that section measured
    // correct. osTickISR early-outs until schedulerRunning is set, so ticks
    // landing before the scheduler is armed are simply counted.
    tickTimer.begin(osTickISR, tickMicros);

    // Remember which channel we got, so the ISR can clear its own flag.
    for (int i = 0; i < 4; i++)
        if (IMXRT_PIT_CHANNELS[i].TCTRL & 1) { osPitCh = i; break; }

    // Now mask interrupts to arm the scheduler atomically: osSchedule() pends
    // PendSV, and it must not fire until osNext is chosen and valid.
    __asm volatile("cpsid i");

    osCurrent = nullptr;                             // "nothing to save on first switch"
    osNext    = nullptr;
    schedulerRunning = true;
    osSchedule();                                    // choose the first task
    if (!osNext) { tickTimer.end(); schedulerRunning = false;
                   __asm volatile("cpsie i"); return; }

    ICSR = ICSR_PENDSVSET;                           // pend the first switch
    __asm volatile("cpsie i");                       // PendSV fires -> first task runs

    for (;;) { }                                     // this MSP context is abandoned
}
