#include <SoftwareSerial.h>

const int PIN_AIN1 = 5, PIN_AIN2 = 6, PIN_PWMA = 11;
const int PIN_BIN1 = 9, PIN_BIN2 = 8, PIN_PWMB = 10;
<<<<<<< HEAD

SoftwareSerial lsa_serial(4, 7);

float kp = 1.5f, ki = 0.05f, kd = 1.0f;
const float BASE_SPEED = 100.0f, SMOOTHING = 0.7f;
const int SENSOR_WEIGHTS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
=======
const int pins[] = {PIN_AIN1, PIN_AIN2, PIN_PWMA, PIN_BIN1, PIN_BIN2, PIN_PWMB};
SoftwareSerial lsa_serial(4, 7);

float kp = 800.0f, ki = 0.0f, kd = 0.2f;
const float BASE_SPEED = 100.0f, SMOOTHING = 0.7f;
const int SENSOR_WEIGHTS[6] = {-5, -3, -1, 1, 3, 5};
>>>>>>> 4323b3e (updated)

float integral = 0.0f, last_error = 0.0f, last_correction = 0.0f;
unsigned long last_time = 0;

<<<<<<< HEAD
bool read_raw_sensors(int s[8]) {
    unsigned long start = millis();
    while (millis() - start < 150) {
        if (lsa_serial.available() && lsa_serial.read() == 0x00) {
=======
bool read_raw_sensors(int s[6]) {
    unsigned long start = millis();
    while (millis() - start < 150) {
        if (lsa_serial.available() && lsa_serial.read() == 0x00) {
            uint8_t buf[8];
>>>>>>> 4323b3e (updated)
            for (int i = 0; i < 8; i++) {
                unsigned long bt = millis();
                while (!lsa_serial.available()) {
                    if (millis() - bt > 50) return false;
                }
<<<<<<< HEAD
                s[i] = lsa_serial.read();
            }
=======
                buf[i] = lsa_serial.read();
            }
            for (int i = 0; i < 6; i++) s[i] = buf[i + 1];
>>>>>>> 4323b3e (updated)
            return true;
        }
    }
    return false;
}

<<<<<<< HEAD
bool get_position(int s[8], float &pos) {
=======
bool get_position(int s[6], float &pos) {
>>>>>>> 4323b3e (updated)
    long w_sum = 0, t_sum = 0;
    for (int i = 0; i < 6; i++) {
        w_sum += (long)s[i] * SENSOR_WEIGHTS[i];
        t_sum += s[i];
    }
    if (t_sum == 0) return false;
    pos = (float)w_sum / (float)t_sum / 2.0f;
    return true;
}

void print_sensors(int s[6], float pos, float error) {
    Serial.print("S: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(s[i]);
        if (i < 5) Serial.print(" ");
    }
    Serial.print("  pos: ");
    Serial.print(pos, 3);
    Serial.print("  err: ");
    Serial.println(error, 3);
}

void set_motors(int left, int right) {
<<<<<<< HEAD
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, constrain(left, 0, 255));
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, constrain(right, 0, 255));
=======
    digitalWrite(PIN_AIN1, left  >= 0 ? HIGH : LOW);
    digitalWrite(PIN_AIN2, left  >= 0 ? LOW  : HIGH);
    analogWrite(PIN_PWMA, constrain(abs(left),  0, 255));

    digitalWrite(PIN_BIN1, right >= 0 ? HIGH : LOW);
    digitalWrite(PIN_BIN2, right >= 0 ? LOW  : HIGH);
    analogWrite(PIN_PWMB, constrain(abs(right), 0, 255));
>>>>>>> 4323b3e (updated)
}

void setup() {
    Serial.begin(115200);
    lsa_serial.begin(9600);
<<<<<<< HEAD
    for (int p : {PIN_AIN1, PIN_AIN2, PIN_PWMA, PIN_BIN1, PIN_BIN2, PIN_PWMB})
        pinMode(p, OUTPUT);
=======
    for (int p : pins) pinMode(p, OUTPUT);
>>>>>>> 4323b3e (updated)
    last_time = micros();
}

void loop() {
<<<<<<< HEAD
    int s[8];
    float error = last_error;

    if (read_raw_sensors(s)) {
        float pos;
        if (get_position(s, pos)) error = -pos;
=======
    int s[6];
    float error = last_error;
    float pos = 0.0f;

    if (read_raw_sensors(s)) {
        if (get_position(s, pos)) {
            error = -pos;
            print_sensors(s, pos, error);
        }
>>>>>>> 4323b3e (updated)
    }

    unsigned long now = micros();
    float dt = (float)(now - last_time) / 1e6f;
<<<<<<< HEAD
    
    if (dt > 0.1f) dt = 0.1f; // no clue what this does
    last_time = now;
=======
    if (dt > 0.1f) dt = 0.1f;
    last_time = now;

    integral = constrain(integral + error * dt, -100.0f, 100.0f);
    float raw = (kp * error) + (ki * integral) + (kd * (error - last_error) / dt);
    float correction = (SMOOTHING * raw) + ((1.0f - SMOOTHING) * last_correction);
    set_motors(BASE_SPEED - correction, BASE_SPEED + correction);
>>>>>>> 4323b3e (updated)

    integral = constrain(integral + error * dt, -100.0f, 100.0f);
    float raw = (kp * error) + (ki * integral) + (kd * (error - last_error) / dt);
    float correction = (SMOOTHING * raw) + ((1.0f - SMOOTHING) * last_correction);

    set_motors(BASE_SPEED + correction, BASE_SPEED - correction);
    last_error = error;
    last_correction = correction;
}