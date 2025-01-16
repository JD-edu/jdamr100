/*
하드웨어 연결 
arduino uno     ESP32 Devkit V1

GND             GND 
7               TX2(17)
8               RX2(16)

*/

#include <SoftwareSerial.h>

// 소프트웨어 시리얼 핀 설정 (예: RX = 10, TX = 11)
SoftwareSerial mySerial(7, 8); // RX, TX

#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

void backward(int R, int L) {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}

void forward(int R, int L) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}

void turnLeft(int R, int L) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, HIGH);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}

void turnRight(int R, int L) {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}

void stopAll() {
  digitalWrite(motor_A_enable, 0);
  digitalWrite(motor_B_enable, 0);
  analogWrite(motor_A, LOW);
  analogWrite(motor_B, LOW);
}


void setup() {
  // 기본 하드웨어 시리얼 포트 (USB 시리얼 모니터)
  Serial.begin(115200);

  // 소프트웨어 시리얼 포트
  mySerial.begin(9600);
  Serial.println("Software Serial에서 데이터 수신 준비 완료");
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}

int motor_r = 0, motor_l = 0;
void loop() {
  // SoftwareSerial에서 데이터가 있을 경우 처리
  if (mySerial.available()) {
    // 수신된 데이터 읽기
    String inString = mySerial.readStringUntil('\n');  // '\n' 문자를 만날 때까지 읽기
    motor_r = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
    motor_l = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();
         
    Serial.print(motor_r);
    Serial.print("  ");
    Serial.println(motor_l);
    forward(motor_r, motor_l);
   
  }
}