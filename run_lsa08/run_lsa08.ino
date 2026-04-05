#include <SoftwareSerial.h>

SoftwareSerial lsaSerial(11, 12);

const int UEN_PIN = /* insert pin here */ ;  // Connect to LSA08 UEN pin; pull LOW to stream data

int sensorValues[8];   // Raw analog value 0-255 for each of the 8 sensors

bool readRawSensors() {
    // UART Mode 3 packet format: [0x00][S0][S1][S2][S3][S4][S5][S6][S7]
    unsigned long t = millis();
    while (true) {
        if (millis() - t > 100) return false;  // timeout
        if (lsaSerial.available() && lsaSerial.peek() == 0x00) {
            lsaSerial.read();
            break;
        } else if (lsaSerial.available()) {
            lsaSerial.read();
        }
    }

    // Read 8 sensor bytes
    t = millis();
    while (lsaSerial.available() < 8) {
        if (millis() - t > 50) return false;
    }
    for (int i = 0; i < 8; i++) {
        sensorValues[i] = (int) lsaSerial.read();  // integer 0-255
    }
    return true;
}

void setup() {
    Serial.begin(9600);
    lsaSerial.begin(19200);  // Match LSA08 BAUDRATE setting (38400 = setting 2)

    pinMode(UEN_PIN, OUTPUT);
    digitalWrite(UEN_PIN, LOW);  // Pull UEN low to enable streaming
}

void loop() {
    if (readRawSensors()) {
        for (int i = 0; i < 8; i++) {
            Serial.print("S"); Serial.print(i);
            Serial.print("="); Serial.print(sensorValues[i]);
            if (i < 7) Serial.print("  ");
        }
        Serial.println();
    }
}