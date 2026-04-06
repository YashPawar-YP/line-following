#include <SoftwareSerial.h>

const int PIN_AIN1 = 5, PIN_AIN2 = 6, PIN_PWMA = 11;
const int PIN_BIN1 = 9, PIN_BIN2 = 8, PIN_PWMB = 10;

SoftwareSerial lsa_serial(4, 7);

float kp = 1.5f, ki = 0.05f, kd = 1.0f;
const float BASE_SPEED = 100.0f, SMOOTHING = 0.7f;
const int SENSOR_WEIGHTS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

float integral = 0.0f, last_error = 0.0f, last_correction = 0.0f;
unsigned long last_time = 0;

bool read_raw_sensors(int s[8]) {
    unsigned long start = millis();
    while (millis() - start < 150) {
        if (lsa_serial.available() && lsa_serial.read() == 0x00) {
            for (int i = 0; i < 8; i++) {
                unsigned long bt = millis();
                while (!lsa_serial.available()) {
                    if (millis() - bt > 50) return false;
                }
                s[i] = lsa_serial.read();
            }
            return true;
        }
    }
    return false;
}

bool get_position(int s[8], float &pos) {
    long w_sum = 0, t_sum = 0;
    for (int i = 0; i < 8; i++) {
        w_sum += (long)s[i] * SENSOR_WEIGHTS[i];
        t_sum += s[i];
    }
    if (t_sum == 0) return false;
    pos = (float)w_sum / (float)t_sum / 2.0f;
    return true;
}

void set_motors(int left, int right) {
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, constrain(left, 0, 255));
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, constrain(right, 0, 255));
}

void setup() {
    Serial.begin(115200);
    lsa_serial.begin(9600);
    for (int p : {PIN_AIN1, PIN_AIN2, PIN_PWMA, PIN_BIN1, PIN_BIN2, PIN_PWMB})
        pinMode(p, OUTPUT);
    last_time = micros();
}

void loop() {
    int s[8];
    float error = last_error;

    if (read_raw_sensors(s)) {
        float pos;
        if (get_position(s, pos)) error = -pos;
    }

    unsigned long now = micros();
    float dt = (float)(now - last_time) / 1e6f;
    
    if (dt > 0.1f) dt = 0.1f; // no clue what this does
    last_time = now;

    integral = constrain(integral + error * dt, -100.0f, 100.0f);
    float raw = (kp * error) + (ki * integral) + (kd * (error - last_error) / dt);
    float correction = (SMOOTHING * raw) + ((1.0f - SMOOTHING) * last_correction);

    set_motors(BASE_SPEED + correction, BASE_SPEED - correction);
    last_error = error;
    last_correction = correction;
}