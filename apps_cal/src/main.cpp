/*
 * Formula Manipal - APPS / BPS CALIBRATION & BRING-UP  -  Teensy 4.1 (VCU PCB)
 * ===========================================================================
 * Standalone ADC reader for the real VCU board. No kernel, no CAN, no control
 * logic -- deliberately, so this is a trustworthy first bring-up check and a
 * clean way to capture calibration endpoints.
 *
 * PINS (VCU PCB, confirmed 2026-08-29):
 *     APPS1 = A8,  APPS2 = A7,  BPS = A6
 *
 * 12-bit ADC (0..4095) with hardware averaging -- 4x the resolution of the old
 * 10-bit bench setup on a safety signal, for free.
 *
 * WHAT IT DOES
 *   - Reads all three channels continuously.
 *   - Tracks running min / max per channel (the calibration endpoints).
 *   - Prints, ~5 Hz:  raw  min  max  and a live %-travel using the min/max seen.
 *   - Flags a channel pinned at 0 or 4095 (open / short-to-rail).
 *
 * HOW TO CALIBRATE
 *   1. Flash, open serial @115200. Leave pedals RELEASED a moment.
 *   2. Press 'r' to reset min/max (clears startup transients).
 *   3. Sweep the ACCELERATOR fully down and up, a few times.
 *   4. Sweep the BRAKE fully.
 *   5. Read the min (released) and max (pressed) for each channel -- those are
 *      the endpoints to put into the car firmware's calibration.
 *
 * SANITY CHECKS TO CONFIRM BEFORE TRUSTING THE BOARD
 *   - Pressing the accelerator moves BOTH APPS1 and APPS2 (they track together).
 *   - APPS1 and APPS2 do NOT sit on top of each other (FSG T11.8 wants two
 *     different, non-intersecting transfer functions) -- one should read
 *     consistently above the other across the whole travel.
 *   - Brake moves only BPS.
 *   - No channel stuck at 0 or 4095.
 *
 * COMMANDS:  r = reset min/max     h = help
 */

#include <Arduino.h>

// ---- pins (VCU PCB) ----
static const int PIN_APPS1 = A8;
static const int PIN_APPS2 = A7;
static const int PIN_BPS   = A6;

static const int ADC_BITS = 12;          // 0..4095
static const int ADC_MAX  = 4095;

struct Chan {
  const char *name;
  int   pin;
  uint16_t val, lo, hi;
};
static Chan ch[3] = {
  { "APPS1", PIN_APPS1, 0, 0xFFFF, 0 },
  { "APPS2", PIN_APPS2, 0, 0xFFFF, 0 },
  { "BPS",   PIN_BPS,   0, 0xFFFF, 0 },
};

static void resetMinMax() {
  for (auto &c : ch) { c.lo = 0xFFFF; c.hi = 0; }
  Serial.println("# min/max reset -- now sweep the pedals fully");
}

static void help() {
  Serial.println(F("\n=== APPS/BPS calibration & bring-up ==="));
  Serial.println(F("pins: APPS1=A8  APPS2=A7  BPS=A6   (12-bit, 0..4095)"));
  Serial.println(F("  r  reset min/max (do this, then sweep pedals fully)"));
  Serial.println(F("  h  help"));
  Serial.println(F("columns per channel:  raw  [min..max]  travel%\n"));
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(ADC_BITS);
  analogReadAveraging(16);               // smooth the reading; endpoints unaffected
  pinMode(PIN_APPS1, INPUT);
  pinMode(PIN_APPS2, INPUT);
  pinMode(PIN_BPS,   INPUT);

  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 1500) {}
  help();
  resetMinMax();
}

void loop() {
  // console
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') resetMinMax();
    else if (c == 'h') help();
  }

  // sample
  for (auto &c : ch) {
    c.val = (uint16_t)analogRead(c.pin);
    if (c.val < c.lo) c.lo = c.val;
    if (c.val > c.hi) c.hi = c.val;
  }

  // print ~5 Hz
  static uint32_t last = 0;
  if (millis() - last >= 200) {
    last = millis();
    char line[220]; int n = 0;
    for (auto &c : ch) {
      uint16_t span = (c.hi > c.lo) ? (c.hi - c.lo) : 0;
      int pct = span ? (int)((long)(c.val - c.lo) * 100 / span) : 0;
      const char *flag = (c.val <= 1) ? " !LOW" : (c.val >= ADC_MAX - 1) ? " !HIGH" : "";
      n += snprintf(line + n, sizeof(line) - n,
                    "%s %4u [%4u..%4u] %3d%%%s   ",
                    c.name, c.val, (c.lo == 0xFFFF ? 0 : c.lo), c.hi, pct, flag);
    }
    Serial.println(line);
  }
}
