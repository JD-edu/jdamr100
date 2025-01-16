#define RXD2 16
#define TXD2 17


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  // 시리얼 모니터 시작
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("hello world");
  Serial2.println("hello 2 world");
  delay(1000);

}
