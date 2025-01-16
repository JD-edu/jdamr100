int ir1 = 7;
int ir2 = 8;

void setup() {
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  Serial.begin(115200);
}

void loop() {
 
  int state = digitalRead(ir1);
  Serial.print("ir1 = ");
  Serial.println(state);

  state = digitalRead(ir2);
  Serial.print("ir2 = ");
  Serial.println(state);

  delay(100);
}