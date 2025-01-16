#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

void setup() {
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);

}
void loop() {

  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, 255);
  analogWrite(motor_B, 255);
  delay(1000);
 
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, 127);
  analogWrite(motor_B, 127);
  delay(1000);

  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, 0);
  analogWrite(motor_B, 0);
  delay(1000);
}