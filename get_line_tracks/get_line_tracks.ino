#include <SoftwareSerial.h>

// LSA08 UART Pins
SoftwareSerial lsaSerial(4, 7);

// Motor Driver Pins
const int ENA = 11;
const int IN1 = 5;
const int IN2 = 6;
const int IN3 = 9;
const int IN4 = 8;
const int ENB = 10;

// Motor Speed (0-255)
const int motorSpeed = 150; 

bool readRawSensors(int* s) {
  while (lsaSerial.available()) lsaSerial.read(); // flush

  unsigned long t = millis();
  while (millis() - t < 150) {
    if (lsaSerial.available() && lsaSerial.read() == 0x00) goto read_bytes;
  }
  return false;

read_bytes:
  t = millis();
  for (int i = 0; i < 8; i++) {
    while (!lsaSerial.available()) if (millis() - t > 50) return false;
    s[i] = lsaSerial.read();
    t = millis();
  }
  return true;
}

void moveForward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setup() {
  Serial.begin(115200);
  lsaSerial.begin(9600);
  
  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(1000); // Delay to let you place the robot on the track
  Serial.println("Starting Data Collection...");
}

void loop() {
  // 1. Move for a short pulse
  moveForward();
  delay(200); 
  
  // 2. Stop to take a clean reading (reduces noise/vibration)
  stopMotors();
  delay(50); 

  // 3. Read and print sensor data
  int s[8];
  if (readRawSensors(s)) {
    for (int i = 0; i < 8; i++) {
      Serial.print(s[i]); 
      if (i < 7) Serial.print(","); // Using commas makes it easy to copy into Excel/Sheets
    }
    Serial.println();
  } else {
    Serial.println("timeout");
  }

  delay(500); // Wait before the next movement step
}