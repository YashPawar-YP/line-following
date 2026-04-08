#include <SoftwareSerial.h>

// Pins (Defined for reference, although only Serial is used for calibration command)
const int AIN1 = 10, AIN2 = 8, PWMA = 5;
const int BIN1 = 6, BIN2 = 9, PWMB = 7;

SoftwareSerial lsa_serial(3, 2); // RX = 3, TX = 2

void setup() {
    Serial.begin(115200);
    lsa_serial.begin(9600);
    
    // Set motor pins to output to ensure they are defined, 
    // though we won't move them unless requested.
    int pins[] = {AIN1, AIN2, PWMA, BIN1, BIN2, PWMB};
    for (int p : pins) pinMode(p, OUTPUT);

    Serial.println(F("====================================="));
    Serial.println(F("   LSA08 Sensor Calibration Utility  "));
    Serial.println(F("====================================="));
    Serial.println(F("Type 'C' and press Enter to start calibration."));
}

void performCalibration() {
    Serial.println(F("\nCalibration started: Sweeping..."));
    Serial.println(F("Please move the sensor across the line manually if needed."));
    
    // Command the LSA08 to perform calibration immediately
    // Address: 0x01, Command: 0x43 ('C'), Data: 0x00, Checksum: 0x44
    lsa_serial.write((byte)0x01);
    lsa_serial.write((byte)0x43);
    lsa_serial.write((byte)0x00);
    lsa_serial.write((byte)0x44);

    // Progress bar or wait
    for(int i = 0; i < 10; i++) {
        delay(1000);
        Serial.print(F("."));
    }
    
    Serial.println(F("\nCalibration finished!"));
    Serial.println(F("You can now upload the main PID code."));
    Serial.println(F("\nType 'C' to calibrate again."));
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'C' || cmd == 'c') {
            performCalibration();
        }
    }
}
