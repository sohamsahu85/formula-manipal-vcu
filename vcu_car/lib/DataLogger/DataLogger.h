/*
 * DataLogger - fail-safe CSV logging to the Teensy 4.1 built-in microSD.
 * Logs the control state + the DBC-decoded DTI550 feedback struct.
 *   - begin() picks the next free "<base>N.csv" and writes the header.
 *   - logRow() writes one row; SD buffers it, we flush at most once/sec.
 *   - Never blocks; if the card is absent/fails, ok() stays false and the caller
 *     keeps running (logging is optional, never safety-critical).
 */
#pragma once
#include <Arduino.h>
#include <SD.h>
#include "DTI550.h"

class DataLogger {
public:
    bool begin(const char* baseName, int csPin = BUILTIN_SDCARD);

    void logRow(uint32_t t_ms,
                uint16_t appsADC, float pedalPct,
                uint16_t bpsADC,  float brakePct,
                bool bppc, bool canStale, bool invFault,
                float torqueCmd,
                const DTI550_Data& d);

    bool ok() const { return ok_; }

private:
    File     file_;
    bool     ok_ = false;
    uint32_t lastFlushMs_ = 0;
    static constexpr uint32_t FLUSH_MS = 1000;
};
