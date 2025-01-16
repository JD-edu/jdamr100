#include <SoftwareSerial.h>

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(7, 8); // RX, TX

void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
}

void loop() {
  // 소프트웨어 시리얼로 데이터가 수신되었는지 확인
  if (mySerial.available()) {
    char incomingByte = mySerial.read();  // 수신한 데이터 읽기
    Serial.print("수신한 데이터: ");
    Serial.println(incomingByte);
  }

  // 100ms 지연
  delay(100);
}
