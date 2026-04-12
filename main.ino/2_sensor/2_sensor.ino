#include "esp32-hal-ledc.h"

//sensor pins - only 2 sensors (left and right)
const int sensor_pins[] = {15, 21};  // S1=15 (left), S5=21 (right)
int sensor_max[] = {4095, 4095};
int sensor_min[] = {4095, 4095};

//led pin
#define led_pin 35

//motor pins
#define AN1 27
#define AN2 26
#define BN1 13
#define BN2 25
#define pwm1 33
#define pwm2 12
#define stby 32

//pwm const
#define pwm_freq 7000
#define pwm_bits 8

//pid
float Kp = 35.0;   // reduce robot oscillations
float Ki = 0.0;    // adds tiny errors steady drift
float Kd = 20.0;   // check nature of error
float last_error = 0;
float integral = 0;

//speed
int max_speed = 230, base_speed = 150, min_speed = 0;

//motors
void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(AN1, HIGH);
    digitalWrite(AN2, LOW);
  }
  else {
    digitalWrite(AN1, LOW);
    digitalWrite(AN2, HIGH);
  }
  ledcWrite(pwm1, abs(speed));
}

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(BN1, HIGH);
    digitalWrite(BN2, LOW);
  }
  else {
    digitalWrite(BN1, LOW);
    digitalWrite(BN2, HIGH);
  }
  ledcWrite(pwm2, abs(speed));
}

void stopMotor() {
  digitalWrite(stby, LOW);
  ledcWrite(pwm1, 0);
  ledcWrite(pwm2, 0);
}

//calc position
float calcPosition(int arr[2]) {
  
  long left_value = arr[0];
  long right_value = arr[1];
  long total = left_value + right_value;
  
  if (total < 100) {  // line lost
    return last_error > 0 ? 1.0 : -1.0;
  }
  
  // Calculate position: -1 = line on left sensor only
  //                  0 = line centered between sensors
  //                  +1 = line on right sensor only
  float position = (float)(right_value - left_value) / total;
  
  // Constrain to -1 to 1 range
  if (position > 1.0) position = 1.0;
  if (position < -1.0) position = -1.0;
  
  return position;
}

//calibrate
void calibration() {
  Serial.println("Calibrating... move bot over line");
  digitalWrite(led_pin, HIGH);
  digitalWrite(stby, HIGH);
  
  // Test sensor readings
  Serial.println("Sensor readings during calibration:");
  
  unsigned long int start = millis();
  while (millis() - start < 3000) {
    for (int i = 0; i < 2; i++) {
      int val = analogRead(sensor_pins[i]);
      Serial.printf("S%d: %d ", i+1, val);  // Print raw values
      if (val > sensor_max[i]) sensor_max[i] = val;
      if (val < sensor_min[i]) sensor_min[i] = val;
    }
    Serial.println();
    
    //slow spin
    setLeftMotor(80);
    setRightMotor(-80);
    delay(100);  // Increased delay to see serial output
  }

  stopMotor();
  digitalWrite(led_pin, LOW);
  
  Serial.println("Calibration done..");
  for (int i = 0; i < 2; i++) {
    Serial.printf("S%d: min: %d max: %d\n", i + 1, sensor_min[i], sensor_max[i]);
  }
}

//sensor read
void ReadSensor(int arr[2]) {
  for (int i = 0; i < 2; i++) {
    int raw = analogRead(sensor_pins[i]);
    arr[i] = map(raw, sensor_min[i], sensor_max[i], 0, 1000);
    arr[i] = constrain(arr[i], 0, 1000);
  }
}

void setup() {
  Serial.begin(115200);

  //sensor pins
  for (int i = 0; i < 2; i++) {
    pinMode(sensor_pins[i], INPUT);
  }

  //motor direction pins
  pinMode(AN1, OUTPUT);
  pinMode(AN2, OUTPUT);
  pinMode(BN1, OUTPUT);
  pinMode(BN2, OUTPUT);
  pinMode(stby, OUTPUT);
  pinMode(led_pin, OUTPUT);

  //start with motors disabled
  digitalWrite(stby, LOW);

  //esp32 ledcAttach(PIN, FREQUENCY, RESOLUTION)
  ledcAttach(pwm1, pwm_freq, pwm_bits);
  ledcAttach(pwm2, pwm_freq, pwm_bits);

  delay(5000);  // place bot for calibration
  calibration();
  delay(2000);

  Serial.println("Starting...");
}

void loop() {
  digitalWrite(stby, HIGH); //motors start
  
  int sensors[2];
  ReadSensor(sensors);

  float position = calcPosition(sensors);
  float error = position;

  integral += error;
  integral = constrain(integral, -300, 300);  //anti-windup

  float derivative = error - last_error;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  last_error = error;  //rewrites new error

  int left_speed = base_speed + (int)correction;
  int right_speed = base_speed - (int)correction;

  left_speed = constrain(left_speed, min_speed, max_speed);
  right_speed = constrain(right_speed, min_speed, max_speed);

  setLeftMotor(left_speed);
  setRightMotor(right_speed);

  Serial.printf("Pos:%.2f, Correction:%.2f, Error:%.2f, Integral:%.2f, Derivative:%.2f, LeftSpeed:%d, RightSpeed:%d\n",
                position, correction, error, integral, derivative, left_speed, right_speed);
}
