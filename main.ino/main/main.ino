#include "esp32-hal-ledc.h"

//sensor pins
const int sensor_pins[] = {23, 13, 16, 26, 24}; 
int sensor_max[] = {4095, 4095, 4095, 4095, 4095};
int sensor_min[] = {0, 0, 0, 0, 0};

//pwm pins
#define pwm1 30 
#define pwm2 28
#define dir1 31 
#define dir2 33 

//pwm const
#define pwm_freq 7000
#define pwm_bits 8

//pid
float Kp = 25.0;   // reduce robot oscillations
float Ki = 0.0;    // adds tiny errors steady drift
float Kd = 15.0;   // check nature of error
float last_error = 0;
float integral = 0;

//speed
int max_speed = 230, base_speed = 150, min_speed = 0;

#define led_pin 16

//position calc
float calcPosition(int arr[5]) {
  const int weights[] = {-2, -1, 0, 1, 2};
  long weighted_sum = 0, total_sum = 0;

  for (int i = 0; i < 5; i++) {
    weighted_sum += (long)arr[i] * weights[i];
    total_sum += arr[i];
  }

  if (total_sum < 100) {  // on white line
    // total is very small so it's like not detecting any line 
    return last_error > 0 ? -2.0 : 2.0;
  }
  return (float)weighted_sum / total_sum;
}

void calibration() {
  Serial.println("Calibrating... move bot over line");  
  digitalWrite(led_pin, HIGH);

  unsigned long int start = millis();
  while (millis() - start < 3000) {
    for (int i = 0; i < 5; i++) {
      int val = analogRead(sensor_pins[i]);
      if (val > sensor_max[i]) sensor_max[i] = val;
      if (val < sensor_min[i]) sensor_min[i] = val;
    }
    //slow spin
    setLeftMotor(80);
    setRightMotor(-80);
    delay(10);
  }

  stopMotor();
  digitalWrite(led_pin, LOW);

  Serial.println("Calibration done..");
  for (int i = 0; i < 5; i++) {
    Serial.printf("S%d: min: %d max: %d\n", i+1, sensor_min[i], sensor_max[i]);
  }
}

//sensor read
void ReadSensor(int arr[5]) { 
  for (int i = 0; i < 5; i++) {
    int raw = analogRead(sensor_pins[i]);
    arr[i] = map(raw, sensor_min[i], sensor_max[i], 0, 1000);
    arr[i] = constrain(arr[i], 0, 1000);
  }
}

//motors
void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(dir2, (speed >= 0) ? HIGH : LOW);
  ledcWrite(pwm2, abs(speed));
}

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(dir1, (speed >= 0) ? HIGH : LOW);
  ledcWrite(pwm1, abs(speed));
}

void stopMotor() {
  ledcWrite(pwm1, 0);
  ledcWrite(pwm2, 0);
}

void setup() {
  Serial.begin(115200);
  
  //sensor pins
  for(int i = 0; i < 5; i++) {
    pinMode(sensor_pins[i], INPUT_PULLUP);
  }

  //motor direction pins
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(led_pin, OUTPUT);

  //esp32 ledcAttach(PIN, FREQUENCY, RESOLUTION)
  ledcAttach(pwm1, pwm_freq, pwm_bits);
  ledcAttach(pwm2, pwm_freq, pwm_bits);

  delay(5000);  // place bot for calibration
  calibration();
  delay(2000);

  Serial.println("Starting...");
}

void loop() {
  int sensors[5];
  ReadSensor(sensors);

  float position = calcPosition(sensors);
  float error = position;

  integral += error;
  integral = constrain(integral, -300, 300);  // anti-windup

  float derivative = error - last_error;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  last_error = error;  // rewrites new error

  int left_speed = base_speed + (int)correction;
  int right_speed = base_speed - (int)correction;

  left_speed = constrain(left_speed, min_speed, max_speed);
  right_speed = constrain(right_speed, min_speed, max_speed);

  setLeftMotor(left_speed);
  setRightMotor(right_speed);

  Serial.printf("Pos:%.2f, Correction:%.2f, Error:%.2f, Integral:%.2f, Derivative:%.2f, LeftSpeed:%d, RightSpeed:%d\n", 
                position, correction, error, integral, derivative, left_speed, right_speed);
}
