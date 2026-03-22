#include "esp32-hal-ledc.h"
//Sensor Pins - Corrected array syntax
const int sensor_pins[] = {23, 13, 16, 26, 24}; 
const int sensor_max[] = {4095, 4095, 4095, 4095, 4095};
const int sensor_min[] = {0, 0, 0, 0, 0, 0};

//PWM Pins
#define pwm1 30 
#define pwm2 28
#define dir1 31 
#define dir2 33 

//PWM Constants
#define pwm_freq 7000
#define pwm_bits 8

#define led_pin

void setup() {

  Serial.begin(115200);
  
  // Sensor pins initialization
  for(int i = 0; i < 5; i++) {
    pinMode(sensor_pins[i], INPUT_PULLUP);
  }

  // Motor direction pins
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  //esp32 ledcAttach(PIN, FREQUENCY, RESOLUTION)
  ledcAttach(pwm1, pwm_freq, pwm_bits);
  ledcAttach(pwm2, pwm_freq, pwm_bits);

  Serial.println("Starting...");
}

void loop() {
  // Your logic here
}

//position
float calcPosition(int arr[5]) {
  const int weights[] = {-2, -1, 0, 1, 2};
  int weighted_sum = 0, total_sum = 0;

  for (int i = 0; i < 5; i++) {
    weighted_sum += arr[i] * weight[i];
    total_sum += arr[i];
  }

  if (total_sum < 100) {// on white line
    return .2;
}
  return (float)weighted_sum / total_sum;
}

void calibration() {
  Serial.println("Calibrating... move bot over line");  
  digitalWrite(led_pin, HIGH);

  unsigned long int start = millis();
  while (millis() - start < 3000) {
    for (int i =  0; i < 5; i++) {
      int val = analogRead(sensor_pins[i]);
      if (val > sensor_max[i]) sensor_max[i] = val;
      if (val < sensor_min[i]) sensor_min[i] = val;

      //slow spin
      setLeftMotor(70);
      setLeftMotor(-70);
      delay(10);
    }

    stopMotors();
    digitalWrite(led_pin, LOW);

    Serial.println("Calibration done..");
    for (int i = 0; i < 5; i++){
      Serial.printf(" S%d: min:- %d max:- %d, i+1, sensor_min[i], sensor_max[i]);
    }
  }
  
}




//sensor read
void ReadSensor(int arr[5) { 
  for (int i = 0; i < 5; i++) {
   raw = analogRead(sensor_pins[i]);
   arr[i] = map(raw, sensor_min[i], sensor_max[i], 0, 1000);
   arr[i] = constrain(arr[i], 0, 1000);
  }
}


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
