/*
 * SignalFilter - median-of-N spike rejection + EMA low-pass, and a slew limiter.
 * Header-only (MedianEMA is a template).
 *
 *   MedianEMA<5> f(0.10f);   float y = f.update(analogRead(pin));
 *   cmd = slewLimitRise(target, cmd, maxRisePerTick);
 */
#pragma once
#include <Arduino.h>

// Cascade: median-of-N (rejects impulsive EMI spikes) -> EMA (smooths the rest).
template <uint8_t N>
class MedianEMA {
public:
    explicit MedianEMA(float alpha) : alpha_(alpha) {}

    float update(uint16_t raw) {
        buf_[idx_] = raw;
        idx_ = (idx_ + 1) % N;
        if (count_ < N) count_++;

        uint16_t t[N];                              // median via sorted copy
        for (uint8_t i = 0; i < count_; i++) t[i] = buf_[i];
        for (uint8_t i = 1; i < count_; i++) {
            uint16_t k = t[i]; int8_t j = i - 1;
            while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
            t[j + 1] = k;
        }
        med_ = t[count_ / 2];

        if (!emaInit_) { ema_ = med_; emaInit_ = true; }
        else           ema_ += alpha_ * (med_ - ema_);
        return ema_;
    }

    // The median stage ONLY (spike-rejected, but NOT EMA-smoothed). Use this
    // where fast response matters more than smoothness -- e.g. a plausibility
    // check that must react within a deadline. The median already rejects lone
    // EMI spikes, so it is safe to act on without the EMA's added lag.
    float median() const { return med_; }

private:
    uint16_t buf_[N];
    uint8_t  idx_ = 0, count_ = 0;
    float    ema_ = 0;
    float    med_ = 0;
    bool     emaInit_ = false;
    float    alpha_;              // 0..1 (higher = faster response, noisier)
};

// Limit how fast a command may RISE (%/tick); a drop passes through instantly
// (releasing / cutting torque is always the safe direction).
inline float slewLimitRise(float target, float prev, float maxRise) {
    float d = target - prev;
    if (d > maxRise) d = maxRise;
    return prev + d;
}
