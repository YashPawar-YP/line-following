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

// FSM States
enum RobotState {
    STATE_FOLLOWING_LINE,
    STATE_PATTERN_DETECTED,
    STATE_STOPPED
};

RobotState current_state = STATE_FOLLOWING_LINE;
unsigned long state_timer = 0;

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
    lsa_serial.begin(38400);
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

    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0f;
    if (dt <= 0.0001f) return;
    last_time = now;

    // --- FSM Transitions ---
    if (validRead && current_state == STATE_FOLLOWING_LINE) {
        if (isIntersectionDetected(sensorValues)) {
            current_state = STATE_PATTERN_DETECTED;
            state_timer = millis();
        }
    }

    // --- FSM Actions ---
    switch (current_state) {
        case STATE_FOLLOWING_LINE:
            if (validRead) executePidControl(sensorValues, dt);
            break;

        case STATE_PATTERN_DETECTED:
            applyMotorSpeeds(0, 0);
            if (millis() - state_timer > 1000) {
                integral = 0; last_error = 0;
                current_state = STATE_FOLLOWING_LINE;
            }
            break;

        case STATE_STOPPED:
            applyMotorSpeeds(0, 0);
            break;
    }
}