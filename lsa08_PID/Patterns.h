#ifndef PATTERNS_H
#define PATTERNS_H

#include <Arduino.h>

// ─────────────────────────── Thresholds ───────────────────────────
const int LINE_THRESHOLD      = 50;   // min value for a sensor to count as "active"
const int CROSS_THRESHOLD     = 5;    // >= 5 sensors lit → cross / horizontal bar
const int FULL_HIT_THRESHOLD  = 150;  // strong hit (all sensors uniformly lit)
const int EDGE_THRESHOLD      = 100;  // edge sensor "strongly lit" for sharp turns
const int SHARP_TURN_INNER    = 30;   // opposite-side sensor must be dim

// ───────────── Existing: Cross junction (5-6 sensors lit) ─────────────
inline bool isCrossDetected(int s[6]) {
    int active = 0;
    for (int i = 0; i < 6; i++) {
        if (s[i] > LINE_THRESHOLD) active++;
    }
    return active >= CROSS_THRESHOLD;
}

// ─────── Existing: Horizontal line (ALL 6 sensors strongly lit) ───────
inline bool isHorizontalLineDetected(int s[6]) {
    for (int i = 0; i < 6; i++) {
        if (s[i] < FULL_HIT_THRESHOLD) return false;
    }
    return true;  // every sensor is strongly lit
}

// ────────── Existing: No line (>= 5 sensors dark) ──────────
inline bool isNothingThere(int s[6]) {
    int inactive = 0;
    for (int i = 0; i < 6; i++) {
        if (s[i] < LINE_THRESHOLD) inactive++;
    }
    return inactive >= CROSS_THRESHOLD;  // >= 5 sensors dark
}

// ───────── T-Junction ─────────
// Both outer edges lit AND the center lit → three-way split.
// Distinguished from a full horizontal bar by requiring at least one
// center sensor to be active while both edges are active, but NOT
// all 6 sensors uniformly strong (that's a horizontal bar).
inline bool isTJunctionDetected(int s[6]) {
    // Must not be a solid horizontal bar
    if (isHorizontalLineDetected(s)) return false;

    bool leftEdge  = (s[0] > LINE_THRESHOLD);
    bool rightEdge = (s[5] > LINE_THRESHOLD);
    bool centerActive = (s[2] > LINE_THRESHOLD) || (s[3] > LINE_THRESHOLD);

    int active = 0;
    for (int i = 0; i < 6; i++) {
        if (s[i] > LINE_THRESHOLD) active++;
    }

    // T-junction: both edges + center lit, with >= 4 sensors active
    return leftEdge && rightEdge && centerActive && (active >= 4);
}

// ───────── Sharp Turn Left ─────────
// Left edge sensors (s1/s2) detect line while right side is dark.
// Indicates the line curves sharply to the left.
inline bool isSharpLeftDetected(int s[6]) {
    bool leftEdgeLit  = (s[0] > EDGE_THRESHOLD) || (s[1] > EDGE_THRESHOLD);
    bool rightSideDim = (s[4] < SHARP_TURN_INNER) && (s[5] < SHARP_TURN_INNER);
    bool centerDim    = (s[2] < LINE_THRESHOLD) && (s[3] < LINE_THRESHOLD);
    return leftEdgeLit && rightSideDim && centerDim;
}

// ───────── Sharp Turn Right ─────────
// Right edge sensors (s5/s6) detect line while left side is dark.
// Indicates the line curves sharply to the right.
inline bool isSharpRightDetected(int s[6]) {
    bool rightEdgeLit = (s[4] > EDGE_THRESHOLD) || (s[5] > EDGE_THRESHOLD);
    bool leftSideDim  = (s[0] < SHARP_TURN_INNER) && (s[1] < SHARP_TURN_INNER);
    bool centerDim    = (s[2] < LINE_THRESHOLD) && (s[3] < LINE_THRESHOLD);
    return rightEdgeLit && leftSideDim && centerDim;
}

// ───────── Circle Detection (uses sensor history) ─────────
// A circle produces sustained, consistent curvature: the weighted
// position stays on the same side (left or right) for many consecutive
// frames.  We look at the last N frames in the history buffer.
//
// Returns:  0 = not a circle
//          -1 = circling left  (sustained negative position)
//          +1 = circling right (sustained positive position)
//
// histBuf:  ring buffer [HISTORY_SIZE][6]
// histPos:  current write index into the ring buffer
// histSize: compile-time constant (HISTORY_SIZE)
// weights:  the 6-element SENSOR_WEIGHTS array from main sketch
//
// MIN_CIRCLE_FRAMES: how many consecutive frames must show the same
//                    side bias before we call it a circle.
const int MIN_CIRCLE_FRAMES = 20;
const float CIRCLE_POS_THRESHOLD = 1.5f; // weighted-position magnitude

inline int isCircleDetected(uint8_t histBuf[][6], int histPos,
                            int histSize, const int weights[6]) {
    int leftCount = 0, rightCount = 0;

    for (int n = 0; n < histSize; n++) {
        int idx = (histPos - 1 - n + histSize) % histSize;

        long wSum = 0, tSum = 0;
        for (int i = 0; i < 6; i++) {
            wSum += (long)histBuf[idx][i] * weights[i];
            tSum += histBuf[idx][i];
        }
        if (tSum == 0) return 0;  // invalid frame — bail

        float pos = (float)wSum / (float)tSum;

        if (pos < -CIRCLE_POS_THRESHOLD)      leftCount++;
        else if (pos > CIRCLE_POS_THRESHOLD)   rightCount++;
        else return 0;  // break in the curve — not a circle
    }

    if (leftCount  >= MIN_CIRCLE_FRAMES) return -1; // circling left
    if (rightCount >= MIN_CIRCLE_FRAMES) return  1; // circling right
    return 0;
}

#endif