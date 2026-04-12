#include <SoftwareSerial.h>
#include "Motors.h"
#include "Patterns.h"

SoftwareSerial lsa_serial(4, 7);

// ─────────────────────── PID & Motion Constants ───────────────────────
float kp = 400.0f, ki = 0.0f, kd = 0.1f;
const float SETPOINT = 0.0f, BASE_SPEED = 60.0f, SMOOTHING = 0.5f;
const int SENSOR_WEIGHTS[6] = {-5, -3, -1, 1, 3, 5};

// Sharp-turn PID overrides: tighter, more aggressive
const float SHARP_TURN_SPEED   = 45.0f;  // slower through sharp turns
const float SHARP_TURN_KP      = 600.0f; // higher proportional gain for snapping

// Circle-handling speed reduction (prevent drift-out on curved arcs)
const float CIRCLE_SPEED       = 50.0f;
const float CIRCLE_KP          = 500.0f;

// ─────────────────────── PID State ───────────────────────
float integral = 0.0f, last_error = 0.0f, last_correction = 0.0f;
unsigned long last_time = 0;

// ─────────────────────── Sensor History ───────────────────────
#define HISTORY_SIZE 25
uint8_t sensorHistory[HISTORY_SIZE][6];
int historyPosition = 0;

void saveToHistory(int s[6]) {
    for (int i = 0; i < 6; i++) {
        sensorHistory[historyPosition][i] = (uint8_t)s[i];
    }
    historyPosition = (historyPosition + 1) % HISTORY_SIZE;
}

// ─────────────────────── FSM States ───────────────────────
enum RobotState {
    STATE_FOLLOWING_LINE,
    STATE_CROSS,
    STATE_T_JUNCTION,
    STATE_SHARP_LEFT,
    STATE_SHARP_RIGHT,
    STATE_CIRCLE,
    STATE_GAP,
    STATE_STOPPED
};

RobotState current_state = STATE_FOLLOWING_LINE;
unsigned long state_timer = 0;

// Gap recovery
const unsigned long GAP_DECAY_MS  = 600;
const unsigned long GAP_FREEZE_MS = 3000;
float gap_entry_correction = 0.0f;

// T-junction: drive straight through (same as cross, but longer burst
// to clear the wider junction opening)
const unsigned long T_JUNCTION_DRIVE_MS = 400;

// Sharp-turn timing: how long to apply the hard pivot before resuming PID
const unsigned long SHARP_TURN_PIVOT_MS = 350;

// Circle: boosted PID runs until the curvature streak breaks
int circle_direction = 0; // -1 left, +1 right

// ─────────────────── Sensor Reader ───────────────────
bool readLsaSensorData(int s[6]) {
    static uint8_t buf[8];
    static int idx = -1; // -1 = waiting

    while (lsa_serial.available() > 0) {
        uint8_t b = lsa_serial.read();
        if (idx == -1) {
            if (b == 0x0D) idx = 0;
        } else {
            buf[idx++] = b;
            if (idx == 8) {
                for (int i = 0; i < 6; i++) s[i] = buf[i + 1];
                idx = -1;
                return true;
            }
        }
    }
    return false;
}

// ─────────────────── Weighted Position ───────────────────
bool calculateLinePosition(int s[6], float& pos) {
    long w_sum = 0, t_sum = 0;
    for (int i = 0; i < 6; i++) {
        w_sum += (long)s[i] * SENSOR_WEIGHTS[i];
        t_sum += s[i];
    }
    if (t_sum == 0) return false;
    pos = (float)w_sum / (float)t_sum;
    return true;
}

// ─────────────────── PID Control ───────────────────
// Parameterised so we can use different kp / base speed per state
void executePidControl(int s[6], float dt, float use_kp, float use_base) {
    float pos;
    if (!calculateLinePosition(s, pos)) return;

    float error = SETPOINT - pos;
    if (ki > 0.0f)
        integral = constrain(integral + (error * dt), -100.0f, 100.0f);
    else
        integral = 0;

    float d_term = (error - last_error) / dt;
    float raw = (use_kp * error) + (ki * integral) + (kd * d_term);
    float correction = (SMOOTHING * raw) + ((1.0f - SMOOTHING) * last_correction);
    applyMotorSpeeds(use_base + correction, use_base - correction);

    last_error = error;
    last_correction = correction;
}

// Convenience: default PID with normal kp & base speed
void executePidControl(int s[6], float dt) {
    executePidControl(s, dt, kp, BASE_SPEED);
}

// ═══════════════════════════ SETUP ═══════════════════════════
void setup() {
    Serial.begin(115200);
    lsa_serial.begin(9600);
    initializeMotorHardware();
    last_time = micros();

    Serial.println(F("--- LSA08 PID v2 ---"));
    Serial.print(F("KP: ")); Serial.print(kp);
    Serial.print(F(" KD: ")); Serial.print(kd);
    Serial.print(F(" BASE: ")); Serial.println(BASE_SPEED);
}

// ═══════════════════════════ LOOP ═══════════════════════════
void loop() {
    int sensorValues[6];
    bool validRead = readLsaSensorData(sensorValues);
    if (validRead) saveToHistory(sensorValues);
    
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0f;
    if (dt <= 0.0001f) return;
    last_time = now;

    // ─────── FSM Transitions (only from LINE-FOLLOWING) ───────
    if (validRead && current_state == STATE_FOLLOWING_LINE) {

        // 1) Horizontal bar: all 6 sensors uniformly lit — drive straight
        if (isHorizontalLineDetected(sensorValues)) {
            applyMotorSpeeds(BASE_SPEED, BASE_SPEED);
            return;
        }

        // 2) Sharp turn left (edge-lit on left, right is dark)
        if (isSharpLeftDetected(sensorValues)) {
            current_state = STATE_SHARP_LEFT;
            state_timer = millis();
            Serial.println(F("[PAT] Sharp LEFT"));
            // Immediately start the hard pivot
            applyMotorSpeeds(-SHARP_TURN_SPEED, SHARP_TURN_SPEED);
            return;
        }

        // 3) Sharp turn right
        if (isSharpRightDetected(sensorValues)) {
            current_state = STATE_SHARP_RIGHT;
            state_timer = millis();
            Serial.println(F("[PAT] Sharp RIGHT"));
            applyMotorSpeeds(SHARP_TURN_SPEED, -SHARP_TURN_SPEED);
            return;
        }

        // 4) T-junction (both edges + center, but not horiz bar)
        if (isTJunctionDetected(sensorValues)) {
            current_state = STATE_T_JUNCTION;
            state_timer = millis();
            Serial.println(F("[PAT] T-Junction"));
        }

        // 5) Cross junction (5+ sensors, not horizontal bar)
        else if (isCrossDetected(sensorValues)) {
            current_state = STATE_CROSS;
            state_timer = millis();
            Serial.println(F("[PAT] Cross"));
        }

        // 6) Circle detection via history (sustained curvature)
        int circDir = isCircleDetected(sensorHistory, historyPosition,
                                       HISTORY_SIZE, SENSOR_WEIGHTS);
        if (circDir != 0) {
            circle_direction = circDir;
            current_state = STATE_CIRCLE;
            state_timer = millis();
            Serial.print(F("[PAT] Circle "));
            Serial.println(circDir < 0 ? F("LEFT") : F("RIGHT"));
        }

        // 7) Line lost
        if (isNothingThere(sensorValues)) {
            gap_entry_correction = last_correction;
            current_state = STATE_GAP;
            state_timer = millis();
            Serial.println(F("[PAT] Gap"));
        }
    }

    // Resume from GAP if line reappears
    if (current_state == STATE_GAP && validRead && !isNothingThere(sensorValues)) {
        integral = 0; last_error = 0;
        current_state = STATE_FOLLOWING_LINE;
    }

    // ─────── FSM Actions ───────
    switch (current_state) {

        // ── Normal PID line following ──
        case STATE_FOLLOWING_LINE:
            if (validRead) executePidControl(sensorValues, dt);
            break;

        // ── Cross junction: drive straight through briefly ──
        case STATE_CROSS:
            applyMotorSpeeds(BASE_SPEED, BASE_SPEED);
            if (millis() - state_timer > 300) {
                integral = 0; last_error = 0;
                current_state = STATE_FOLLOWING_LINE;
            }
            break;

        // ── T-junction: drive straight a bit longer to clear it ──
        case STATE_T_JUNCTION:
            applyMotorSpeeds(BASE_SPEED, BASE_SPEED);
            if (millis() - state_timer > T_JUNCTION_DRIVE_MS) {
                integral = 0; last_error = 0;
                current_state = STATE_FOLLOWING_LINE;
            }
            break;

        // ── Sharp left: hard pivot then resume with aggressive PID ──
        case STATE_SHARP_LEFT:
            if (millis() - state_timer < SHARP_TURN_PIVOT_MS) {
                // Still pivoting: reverse left motor, forward right
                applyMotorSpeeds(-SHARP_TURN_SPEED, SHARP_TURN_SPEED);
            } else {
                // Pivot done — if we can see the line, switch to aggressive PID
                if (validRead && !isNothingThere(sensorValues)) {
                    integral = 0; last_error = 0;
                    current_state = STATE_FOLLOWING_LINE;
                }
                // If line still not found, keep pivoting (extend the turn)
                else {
                    applyMotorSpeeds(-SHARP_TURN_SPEED * 0.7f,
                                      SHARP_TURN_SPEED * 0.7f);
                }
                // Safety: give up after 1.5s of pivoting
                if (millis() - state_timer > 1500) {
                    integral = 0; last_error = 0;
                    current_state = STATE_FOLLOWING_LINE;
                }
            }
            break;

        // ── Sharp right: mirror of sharp left ──
        case STATE_SHARP_RIGHT:
            if (millis() - state_timer < SHARP_TURN_PIVOT_MS) {
                applyMotorSpeeds(SHARP_TURN_SPEED, -SHARP_TURN_SPEED);
            } else {
                if (validRead && !isNothingThere(sensorValues)) {
                    integral = 0; last_error = 0;
                    current_state = STATE_FOLLOWING_LINE;
                } else {
                    applyMotorSpeeds(SHARP_TURN_SPEED * 0.7f,
                                     -SHARP_TURN_SPEED * 0.7f);
                }
                if (millis() - state_timer > 1500) {
                    integral = 0; last_error = 0;
                    current_state = STATE_FOLLOWING_LINE;
                }
            }
            break;

        // ── Circle: run PID with boosted kp & reduced speed ──
        case STATE_CIRCLE:
            if (validRead) {
                executePidControl(sensorValues, dt, CIRCLE_KP, CIRCLE_SPEED);

                // Check if curvature streak is still alive
                int dir = isCircleDetected(sensorHistory, historyPosition,
                                           HISTORY_SIZE, SENSOR_WEIGHTS);
                if (dir == 0) {
                    // Streak broken — back to normal PID
                    integral = 0; last_error = 0;
                    current_state = STATE_FOLLOWING_LINE;
                    Serial.println(F("[PAT] Circle END"));
                }
            }
            break;

        // ── Gap: decay last correction, then freeze at center ──
        case STATE_GAP: {
            unsigned long elapsed = millis() - state_timer;
            float correction;
            if (elapsed < GAP_DECAY_MS) {
                float t = (float)elapsed / (float)GAP_DECAY_MS;
                correction = gap_entry_correction * (1.0f - t);
            } else {
                correction = 0.0f;
            }
            applyMotorSpeeds(BASE_SPEED + correction, BASE_SPEED - correction);

            if (elapsed > GAP_FREEZE_MS) {
                current_state = STATE_STOPPED;
            }
            break;
        }

        // ── Stopped ──
        case STATE_STOPPED:
            applyMotorSpeeds(0, 0);
            break;
    }
}