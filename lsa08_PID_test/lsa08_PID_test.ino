#include <SoftwareSerial.h>

// Pins
const int AIN1 = 10, AIN2 = 8, PWMA = 5;
const int BIN1 = 6, BIN2 = 9, PWMB = 7;

SoftwareSerial lsa_serial(3, 2); // RX = 3, TX = 2

// PID Constants
float Kp = 800.0, Ki = 0.01, Kd = 15.0; 
int base_speed = 50;
float last_error = 0, integral = 0;

void setup() {
    Serial.begin(115200);
    lsa_serial.begin(9600);
    
    int pins[] = {AIN1, AIN2, PWMA, BIN1, BIN2, PWMB};
    for (int p : pins) pinMode(p, OUTPUT);

    performCalibration();

    Serial.println(F("--- LSA08 PID Test ---"));
    Serial.print(F("Kp: ")); Serial.print(Kp);
    Serial.print(F(" Ki: ")); Serial.print(Ki);
    Serial.print(F(" Kd: ")); Serial.println(Kd);
}

void drive(int left_speed, int right_speed) {
    digitalWrite(AIN1, left_speed >= 0);
    digitalWrite(AIN2, left_speed < 0);
    analogWrite(PWMA, constrain(abs(left_speed), 0, 255));

    digitalWrite(BIN1, right_speed >= 0);
    digitalWrite(BIN2, right_speed < 0);
    analogWrite(PWMB, constrain(abs(right_speed), 0, 255));
}

void performCalibration() {
    Serial.println(F("Calibration started: Sweeping..."));
    
    // Command the LSA08 to perform calibration immediately
    // Address: 0x01, Command: 0x43 ('C'), Data: 0x00, Checksum: 0x44
    lsa_serial.write((byte)0x01);
    lsa_serial.write((byte)0x43);
    lsa_serial.write((byte)0x00);
    lsa_serial.write((byte)0x44);

    delay(10000);
    Serial.println(F("Calibration finished!"));
    delay(500); // Give user time to see it finished
}

float get_line_error() {
    // Clear overflow if we're falling behind
    if (lsa_serial.available() > 30) {
        while (lsa_serial.available() > 9) lsa_serial.read();
    }

    // Search for header byte (0x0D)
    while (lsa_serial.available() > 0 && lsa_serial.peek() != 0x0D) {
        lsa_serial.read(); 
    }

    // Process frame if enough data is available
    if (lsa_serial.available() >= 9) {
        lsa_serial.read(); // Consume the header 0x0D
        lsa_serial.read(); // Consume Sensor 0 (discarded)
        
        long sum = 0, avg = 0;
        const int SENSOR_WEIGHTS[6] = {-5, -3, -1, 1, 3, 5};

        for (int i = 0; i < 6; i++) {
            int val = lsa_serial.read();
            
            // NOISE GATE: Low values (background noise) are treated as 0
            int reading = (val > 20) ? val : 0; 
            
            sum += (long)reading * SENSOR_WEIGHTS[i];
            avg += reading;
        }
        
        lsa_serial.read(); // Consume Sensor 7 (discarded)
        
        if (avg == 0) return last_error; // Keep last error if line is lost
        return (float)sum / avg; 
    }
    return last_error;
}

void loop() {
    float error = get_line_error();
    
    // Calculate PID 
    float P = error;
    integral += error;
    integral = constrain(integral, -100, 100); 
    float D = error - last_error;

    float correction = (Kp * P) + (Ki * integral) + (Kd * D);
    last_error = error;

    // Logging for Serial Monitor
    Serial.print(F("Err: ")); Serial.print(error);
    Serial.print(F(" | P: ")); Serial.print(P * Kp);
    Serial.print(F(" | I: ")); Serial.print(integral * Ki);
    Serial.print(F(" | D: ")); Serial.print(D * Kd);
    Serial.print(F(" | Corr: ")); Serial.println(correction);

    // Continuously drive the motors
    drive(base_speed + correction, base_speed - correction);
    
    delay(1); 
}