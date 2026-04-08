#ifndef PATTERNS_H
#define PATTERNS_H

#include <Arduino.h>

const int LINE_THRESHOLD      = 50;
// Minimum sensors active to count as "line present"
const int CROSS_THRESHOLD     = 5;   // >= 5 sensors lit → cross / horizontal bar
// How strongly a sensor must be lit to count as "full coverage"
const int FULL_HIT_THRESHOLD  = 150; // tweak for your sensor output range

// Cross junction: 5-6 sensors lit (includes a narrow cross arm)
inline bool isCrossDetected(int s[6]) {
    int active = 0;
    for (int i = 0; i < 6; i++) {
        if (s[i] > LINE_THRESHOLD) active++;
    }
    return active >= CROSS_THRESHOLD;
}

// Horizontal line: ALL 6 sensors reading strongly — solid bar across track.
// A true cross from the side only lights 5; a horizontal bar lights all 6.
inline bool isHorizontalLineDetected(int s[6]) {
    for (int i = 0; i < 6; i++) {
        if (s[i] < FULL_HIT_THRESHOLD) return false;
    }
    return true; // every sensor is strongly lit
}

// No line: 5 or more sensors are dark
inline bool isNothingThere(int s[6]) {
    int inactive = 0;
    for (int i = 0; i < 6; i++) {
        if (s[i] < LINE_THRESHOLD) inactive++;
    }
    return inactive >= CROSS_THRESHOLD; // >= 5 sensors dark
}
    

#endif
