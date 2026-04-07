#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// Pins
const int PIN_AIN1 = 5, PIN_AIN2 = 6, PIN_PWMA = 11;
const int PIN_BIN1 = 9, PIN_BIN2 = 8, PIN_PWMB = 10;
const int motor_pins[] = {PIN_AIN1, PIN_AIN2, PIN_PWMA, PIN_BIN1, PIN_BIN2, PIN_PWMB};

inline void initializeMotorHardware() {
    for (int p : motor_pins) {
        pinMode(p, OUTPUT);
    }
}

inline void applyMotorSpeeds(float left, float right) {
    int l_pwm = constrain((int)roundf(left),  -255, 255);
    int r_pwm = constrain((int)roundf(right), -255, 255);

    digitalWrite(PIN_AIN1, l_pwm >= 0 ? HIGH : LOW);
    digitalWrite(PIN_AIN2, l_pwm >= 0 ? LOW  : HIGH);
    analogWrite(PIN_PWMA, abs(l_pwm));

    digitalWrite(PIN_BIN1, r_pwm >= 0 ? HIGH : LOW);
    digitalWrite(PIN_BIN2, r_pwm >= 0 ? LOW  : HIGH);
    analogWrite(PIN_PWMB, abs(r_pwm));
}

#endif
