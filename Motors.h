#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// Pins
const int AIN1 = 10, AIN2 = 8, PWMA = 5;
const int BIN1 = 6, BIN2 = 9, PWMB = 7;

const int motor_pins[] = {AIN1, AIN2, PWMA, BIN1, BIN2, PWMB};

inline void initializeMotorHardware() {
    for (int p : motor_pins) {
        pinMode(p, OUTPUT);
    }
}

inline void applyMotorSpeeds(float left, float right) {
    int l_pwm = constrain((int)roundf(left),  -255, 255);
    int r_pwm = constrain((int)roundf(right), -255, 255);

    digitalWrite(AIN1, l_pwm >= 0 ? HIGH : LOW);
    digitalWrite(AIN2, l_pwm >= 0 ? LOW  : HIGH);
    analogWrite(PWMA, abs(l_pwm));

    digitalWrite(BIN1, r_pwm >= 0 ? HIGH : LOW);
    digitalWrite(BIN2, r_pwm >= 0 ? LOW  : HIGH);
    analogWrite(PWMB, abs(r_pwm));
}

#endif