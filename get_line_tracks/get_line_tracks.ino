#include <SoftwareSerial.h>

SoftwareSerial lsaSerial(4, 7);

bool readRawSensors(int* s) {
  while (lsaSerial.available()) lsaSerial.read();  // flush stale bytes

  unsigned long t = millis();
  while (millis() - t < 150) {
    if (lsaSerial.available() && lsaSerial.read() == 0x00) goto read_bytes;
  }
  return false; a XZSC

read_bytes:
  t = millis();
  for (int i = 0; i < 8; i++) {
    while (!lsaSerial.available()) if (millis() - t > 50) return false;
    s[i] = lsaSerial.read();
    t = millis();
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  lsaSerial.begin(9600);
  delay(200);
  Serial.println("ready");
}

void loop() {
  int s[8];
  if (readRawSensors(s)) {
    for (int i = 0; i < 8; i++) {
      Serial.print("S"); Serial.print(i);
      Serial.print("="); Serial.print(s[i]);
      if (i < 7) Serial.print("  ");
    }
    Serial.println();
  } else {
    Serial.println("timeout");
  }
}