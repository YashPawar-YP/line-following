#ifndef PATTERNS_H
#define PATTERNS_H

#include <Arduino.h>

const int LINE_THRESHOLD = 100;
const int ACTIVE_SENSOR_COUNT_THRESHOLD = 5;

// Function to analyze patterns (Example: Stop if all sensors see the line)
inline bool isIntersectionDetected(int s[6]) {
    int active_sensors = 0;
    for (int i = 0; i < 6; i++) {
        if (s[i] > LINE_THRESHOLD) { 
            active_sensors++;
        }
    }
    return active_sensors >= ACTIVE_SENSOR_COUNT_THRESHOLD;
}

#endif
