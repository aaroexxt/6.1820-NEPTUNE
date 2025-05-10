// FreqThreshold.h
// Author: Aaron Becker 4/13/25

#ifndef FREQ_THRESHOLD_H
#define FREQ_THRESHOLD_H

/**************** FREQUENCY THRESHOLD LIBRARY */

#include <Arduino.h>
#include "Protocol.h"

// User-configurable
#define FREQ_MEASURE_DURATION_MS 5000
#define THRESHOLD_MULT_FACTOR 5.25

#define MAX_THRESHOLDS 10

struct FreqThreshold {
    float freq;
    float amp;
};

FreqThreshold ampThresholds[MAX_THRESHOLDS] = {
    // freq, amplitude thresh (from TestBandpassFiltering)
    {MESSAGE_0_FREQ, 3250}, // 7.5 kHz
    {MESSAGE_START_FREQ, 3500}, // 8 kHz
    {MESSAGE_1_FREQ, 3000}, // 8.5 kHz
    {MESSAGE_END_FREQ, 2500} // 9 kHz
};
int numThresholds = 4;

// Returns amplitude threshold for a given frequency (Hz)
int16_t getAmplitudeThreshold(float freqHz) {
    if (numThresholds == 0) return 1000;

    // Clamp to bounds
    if (freqHz <= ampThresholds[0].freq) return ampThresholds[0].amp;
    if (freqHz >= ampThresholds[numThresholds - 1].freq) return ampThresholds[numThresholds - 1].amp;

    // Find interval
    for (int i = 0; i < numThresholds - 1; i++) {
        float f1 = ampThresholds[i].freq;
        float f2 = ampThresholds[i + 1].freq;
        if (freqHz >= f1 && freqHz <= f2) {
            float a1 = ampThresholds[i].amp;
            float a2 = ampThresholds[i + 1].amp;
            float t = (freqHz - f1) / (f2 - f1);
            return (int16_t)(a1 * (1 - t) + a2 * t);
        }
    }

    return 1000;
}

#endif // FREQ_THRESHOLD_H