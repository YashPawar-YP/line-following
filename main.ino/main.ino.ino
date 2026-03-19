//Sensor Pin
const int sensor_pins = {23, 13 ,16, 26, 24};
  
//PWM Pin
#define pwm1 30 //pin for speed
#define pwm2 28
#define dir1 31 //pin for direction
#define dir2 33 

//sensor speed
int sensorMin[5] = {4095, 4095, 4095, 4095, 4095};
int sensorMax[5] = {0, 0, 0, 0, 0};

//motor speed
int max_speed = 220;
int min_speed = 0;
int base_speed = 130;

//PWM
#define pwm_freq 7000
#define pwm_bits 8
#define ch left
#define ch_right

void setup() {
  
  //sensor pins
  for(int i = 0; i < 5; i++) {
    pinMode(sensor_pins[i], INPUT_PULLUP);
    }

  //motor pins
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  //esp32
  ledAttachPin(pwm1, ch_left);
  ledAttachPin(pwm2, ch_right);
  ledcSetup(ch_left, pwm_freq, pwm_bits);
  ledcSetup(ch_right, pwm_freq, pwm_bits);
  
}

void loop() {


}

// motor
void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed >= 0) digitalWrite(dir1, HIGH);
  else digitalWrite(dir2, LOW);

  ledcWrite(ch_right, abs(speed));
}

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed >= 0) digitalWrite(dir1, HIGH);
  else digitalWrite(dir1, LOW);

  ledcWrite(ch_left, abs(speed));
}

void stopMotor() {
  leddcWrite(ch_left, 0);
  ledcWrite(ch_right, 0);
}
}
