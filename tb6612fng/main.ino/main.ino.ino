#include "esp32-hal-ledc.h"
// ===== Pin Definitions =====
#define AN1 27
#define AN2 26
#define BN1 13
#define BN2 25
#define PWM1 33 // Right motor
#define PWM2 12 // Left motor
#define STBY 32

// ===== PWM Settings =====
#define PWM_FREQ 5000
#define PWM_RES 8 // 0–255
int speedR = 200;
int speedL = 200;

// ===== Movement =====
void moveForward() {
  digitalWrLOWite(AN1, LOW);
  digitalWrite(AN2, HIGH);
  ledcWrite(PWM1, speedR); // Write directly to the PIN

  digitalWrite(BN1, HIGH);
  digitalWrite(BN2, LOW);
  ledcWrite(PWM2, speedL); // Write directly to the PIN
}

void moveBackward() {
  digitalWrite(AN1, HIGH);
  digitalWrite(AN2, LOW);
  ledcWrite(PWM1, speedR);

  digitalWrite(BN1, LOW);
  digitalWrite(BN2, HIGH);
  ledcWrite(PWM2, speedL);
}

void stopMotors() {
  ledcWrite(PWM1, 0);
  ledcWrite(PWM2, 0);
}

void setup() {
  pinMode(AN1, OUTPUT);
  pinMode(AN2, OUTPUT);
  pinMode(BN1, OUTPUT);
  pinMode(BN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // New API: ledcAttach(pin, freq, resolution)
  ledcAttach(PWM1, PWM_FREQ, PWM_RES);
  ledcAttach(PWM2, PWM_FREQ, PWM_RES);
}

void loop() {
  moveForward();
  delay(5000);
  stopMotors();
  delay(2000);
  moveBackward();
  delay(5000);
  stopMotors();
  delay(4000);
}
