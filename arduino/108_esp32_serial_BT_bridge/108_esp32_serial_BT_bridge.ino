#include <BluetoothSerial.h> // ESP32 블루투스 라이브러리

BluetoothSerial SerialBT;  // 블루투스 시리얼 객체 생성
#define RXD2 16  // Arduino로부터 데이터 수신 (ESP32 RX)
#define TXD2 17  // Arduino로 데이터 송신 (ESP32 TX)

bool isConnected = false; // 연결 상태를 저장

void setup() {
  Serial.begin(115200);  // ESP32 기본 시리얼 포트 (디버깅용)
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Arduino와 통신할 시리얼 포트
  SerialBT.begin("ESP32_BT"); // 블루투스 이름 설정
  Serial.println("ESP32 Bluetooth Started");
}

void loop() {
  if (SerialBT.hasClient()) {  // 클라이언트가 연결된 경우
    if (!isConnected) {  // 이전에 연결되지 않았던 경우
      Serial.println("Bluetooth Connected!");
      isConnected = true;
    }
  } else {  // 클라이언트가 연결되지 않은 경우
    if (isConnected) {  // 이전에 연결되었던 경우
      Serial.println("Bluetooth Disconnected!");
      isConnected = false;
    }
  }
  // 1. 블루투스에서 받은 데이터를 Arduino로 전달
  if (SerialBT.available()) {
    String btData = SerialBT.readStringUntil('\n'); // 블루투스에서 문자열 읽기
    Serial.println("Received from Bluetooth: " + btData);
    Serial2.println(btData); // Arduino로 데이터 전송
  }

  // 2. Arduino에서 받은 데이터를 블루투스로 전달
  if (Serial2.available()) {
    String arduinoData = Serial2.readStringUntil('\n'); // Arduino에서 데이터 읽기
    Serial.println("Received from Arduino: " + arduinoData);
    SerialBT.println(arduinoData); // 블루투스로 데이터 전송
  }
}
