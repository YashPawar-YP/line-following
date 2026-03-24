#define AN1
#define AN2
#define AN3
#define AN4
#define PWM1
#define PWM2
#define STBY

void move_forward_R() {
  digitalWrite(AN1, HIGH);
  digitalWrite(AN2, LOW);
  digitalWrite(PWM1, 500);
}

void move_forward_L() {
  digitalWrite(AN3, HIGH);
  digitalWrite(AN4, LOW);
  digitalWrite(PWM2, 1000);
}

void move_backward_R() {
  digitalWrite(AN1, LOW);
  digitalWrite(AN2, HIGH);
  digitalWrite(PWM1, 500);
}

void move_backward_L() {
  digitalWrite(AN1, LOW);
  digitalWrite(AN2, HIGH);
  digitalWrite(PWM1, 1000);
}


void setup() {
  pinMode(AN1, OUTPUT);
  pinMode(AN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(STBY, OUTPUT);
}

void loop() {

  digitalWrite(STBY, HIGH);

  move_forward_R();
  move_forward_L();
  delay(5000);

  //move backward
  move_backward_R();
  move_backward_L();
  delay(5000);

  //stop
  digitalWrite(STBY, LOW);
  delay(2000);
}
