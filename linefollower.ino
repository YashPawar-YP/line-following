// sensor pins (digital)
const int sensor_pins[] = {34, 35, 32, 26, 25};

// Motor Driver
const int AIN1 = 4;
const int AIN2 = 5;
const int PWMA = 16;

const int BIN1 = 18;
const int BIN2 = 19;
const int PWMB = 23;

const int STBY = 17;

// variables
float Kp = 1.5;
float Ki = 0.05;
float Kd = 1.0;

float integral = 0;
float last_error = 0;
float last_correction = 0;
unsigned long last_time = 0;

const float setpoint = 0;
const float base_speed = 100;
const float A = 0.7;

void setMotorA(int speed, bool forward) {
  digitalWrite(AIN1, forward ? HIGH : LOW);
  digitalWrite(AIN2, forward ? LOW : HIGH);
  analogWrite(PWMA, speed);
}

void setMotorB(int speed, bool forward) {
  digitalWrite(BIN1, forward ? HIGH : LOW);
  digitalWrite(BIN2, forward ? LOW : HIGH);
  analogWrite(PWMB, speed);
}

void readSensor(int arr[5]) {
  for (int i = 0; i < 5; i++) {
    // flip reading, 1 means line and 0 means no line instead of 0 means line and 1 means no line
    arr[i] = (digitalRead(sensor_pins[i]) == LOW) ? 1000 : 0;
  }
}

float getPosition(int arr[5]) {
  const int weights[] = {-2, -1, 0, 1, 2};
  long weighted_sum = 0, total_sum = 0;

  for (int i = 0; i < 5; i++) {
    weighted_sum += (long)arr[i] * weights[i];
    total_sum += arr[i];
  }
  
  return (float)weighted_sum / total_sum;
}


void setup() {

  for (int i = 0; i < 5; i++) {
    pinMode(sensor_pins[i], INPUT);
  }

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH); // wake up from standby mode
  last_time = micros();
}

void loop() {
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0f;

  if (dt <= 0) return; // guard

  int readings[5];
  readSensor(readings);
  float state = getPosition(readings);

  float error = setpoint - state;

  float P = Kp * error;

  integral += error * dt;
  integral = constrain(integral, -100.0f, 100.0f);
  float I = Ki * integral;

  float D = Kd * ((error - last_error) / dt);

  float raw_correction = P + I + D;

  float correction = (A * raw_correction) + ((1.0f - A) * last_correction);

  int left_speed = constrain((int)(base_speed + correction), 0, 255);
  int right_speed = constrain((int)(base_speed - correction), 0, 255);

  setMotorA(left_speed,  true);
  setMotorB(right_speed, true);

  last_error = error;
  last_time = current_time;
  last_correction = correction;
}