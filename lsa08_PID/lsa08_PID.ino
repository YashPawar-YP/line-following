#include <SoftwareSerial.h>
#include "Motors.h"
#include "Patterns.h"

SoftwareSerial lsa_serial(4, 7);

// PID & Motion Constants
float kp = 400.0f, ki = 0.0f, kd = 0.1f;
const float SETPOINT = 0.0f, BASE_SPEED = 60.0f, SMOOTHING = 0.5f;
const int SENSOR_WEIGHTS[6] = {-5, -3, -1, 1, 3, 5};

// PID State
float integral = 0.0f, last_error = 0.0f, last_correction = 0.0f;
unsigned long last_time = 0;

// history
#define HISTORY_SIZE 25
uint8_t sensorHistory[HISTORY_SIZE][6];
int historyPosition = 0;

void saveToHistory(int s[6]) {
    for (int i = 0; i < 6; i++) {
        sensorHistory[historyPosition][i] = (uint8_t)s[i];
    }
    historyPosition = (historyPosition + 1) % HISTORY_SIZE;
}

// FSM States
enum RobotState {
    STATE_FOLLOWING_LINE,
    STATE_CROSS,
    STATE_GAP,
    STATE_STOPPED
};

RobotState current_state = STATE_FOLLOWING_LINE;
unsigned long state_timer = 0;

// Gap recovery: decay the last known correction to zero, then freeze
const unsigned long GAP_DECAY_MS  = 600;  // ms over which correction decays to 0
const unsigned long GAP_FREEZE_MS = 3000; // ms to keep searching before giving up
float gap_entry_correction = 0.0f;        // correction at the moment line was lost

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

void executePidControl(int s[6], float dt) {
    float pos;
    if (!calculateLinePosition(s, pos)) return; // Line lost — hold last output

    float error = SETPOINT - pos;
    if (ki > 0.0f)
        integral = constrain(integral + (error * dt), -100.0f, 100.0f);
    else
        integral = 0;

    float d_term = (error - last_error) / dt;
    float raw = (kp * error) + (ki * integral) + (kd * d_term);
    float correction = (SMOOTHING * raw) + ((1.0f - SMOOTHING) * last_correction);
    applyMotorSpeeds(BASE_SPEED + correction, BASE_SPEED - correction);

    last_error = error;
    last_correction = correction;
}

void setup() {
    Serial.begin(115200);
    lsa_serial.begin(9600);
    initializeMotorHardware();
    last_time = micros();

    Serial.println(F("--- LSA08 PID ---"));
    Serial.print(F("KP: ")); Serial.print(kp);
    Serial.print(F(" KD: ")); Serial.print(kd);
    Serial.print(F(" BASE: ")); Serial.println(BASE_SPEED);
}

void loop() {
    int sensorValues[6];
    bool validRead = readLsaSensorData(sensorValues);
    if (validRead) saveToHistory(sensorValues);
    
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0f;
    if (dt <= 0.0001f) return;
    last_time = now;

    // --- FSM Transitions ---
    if (validRead && current_state == STATE_FOLLOWING_LINE) {
        // Horizontal bar: all 6 sensors fully lit — just drive straight through it
        if (isHorizontalLineDetected(sensorValues)) {
            applyMotorSpeeds(BASE_SPEED, BASE_SPEED);
            return; // skip PID and further state checks this frame
        }
        // Cross junction (5+ sensors, but NOT a full horizontal bar)
        if (isCrossDetected(sensorValues)) {
            current_state = STATE_CROSS;
            state_timer = millis();
        }
        // Line lost
        if (isNothingThere(sensorValues)) {
            gap_entry_correction = last_correction; // snapshot
            current_state = STATE_GAP;
            state_timer = millis();
        }
    }
    // If we were in STATE_GAP and the line comes back, resume immediately
    if (current_state == STATE_GAP && validRead && !isNothingThere(sensorValues)) {
        integral = 0; last_error = 0;
        current_state = STATE_FOLLOWING_LINE;
    }

    // --- FSM Actions ---
    switch (current_state) {
        case STATE_FOLLOWING_LINE:
            if (validRead) executePidControl(sensorValues, dt);
            break;

        case STATE_CROSS:
            applyMotorSpeeds(BASE_SPEED, BASE_SPEED); // drive through at base speed
            if (millis() - state_timer > 300) {        // short burst, then resume
                integral = 0; last_error = 0;
                current_state = STATE_FOLLOWING_LINE;
            }
            break;

        case STATE_GAP: {
            unsigned long elapsed = millis() - state_timer;
            float correction;
            if (elapsed < GAP_DECAY_MS) {
                // Linearly decay the last known correction toward 0
                float t = (float)elapsed / (float)GAP_DECAY_MS; // 0.0 → 1.0
                correction = gap_entry_correction * (1.0f - t);
            } else {
                correction = 0.0f; // frozen at center
            }
            applyMotorSpeeds(BASE_SPEED + correction, BASE_SPEED - correction);

            // Give up and stop if we've been lost too long
            if (elapsed > GAP_FREEZE_MS) {
                current_state = STATE_STOPPED;
            }
            break;
        }

        case STATE_STOPPED:
            applyMotorSpeeds(0, 0);
            break;
    }