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
  for (int x = 50; x < 200; x++) {
    digitalWrite(motor_A_enable, HIGH);
    digitalWrite(motor_B_enable, HIGH);
    analogWrite(motor_A, x);
    analogWrite(motor_B, x);
    delay(10);
  }
  for (int y = 200; y > 50; y--) {
    digitalWrite(motor_A_enable, HIGH);
    digitalWrite(motor_B_enable, HIGH);
    analogWrite(motor_A, y);
    analogWrite(motor_B, y);
    delay(1);
  }
  delay(1000);
}
