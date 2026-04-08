const int AIN1 = 10, AIN2 = 8, PWMA = 5;
const int BIN1 = 6, BIN2 = 9, PWMB = 7;

int robotSpeed = 200;

void setup() {
  // Motor A Pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  

}

void loop() {
  analogWrite(PWMA, robotSpeed);
  analogWrite(PWMB, robotSpeed);
  
  // Motor A Forward
  digitalWrite(AIN1, HIGH); 
  digitalWrite(AIN2, LOW); 
  
  // Motor B Forward
  digitalWrite(BIN1, HIGH); 
  digitalWrite(BIN2, LOW);
}