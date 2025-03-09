# JDAMR100 아두이노 코드

[101_motor_no_speed_control 모터 정/역회전 테스트](#101_motor_no_speed_control)   
[101_motor_variable_speed_control 모터 속도 조절 테스트](#101_motor_variable_speed_control)   
[102_ir_test IR 센서 테스트](#102_ir_test)   
[103_ultrasonic_test 초음파 센서 테스트](#103_ultrasonic_test)   
[104_encoder_go_forward 양쪽 바퀴 엔코더 동작 테스트](#104_encoder_go_forward)   
[105_arduino_serial_comm_with_esp32 아두이노 - ESP32 시리얼 통신(아두이노 수신)](#105_arduino_serial_comm_with_esp32)   
[105_esp32_serial_comm_with_arduino 아두이노 - ESP32 시리얼 통신(esp32 출력)](#105_esp32_serial_comm_with_arduino)   
[106_esp32_i2c_test 아두이노의 소프트 시리얼을 통해 ESP32에서 전송한 정보를 받는 코드](#106_esp32_i2c_test)   
[107_robot_drive_with_encoder ESP32 없이 엔코더를 사용하여 직진](#107_robot_drive_with_encoder)   
[108_arduino_serial_esp32_comm 아두이노 - ESP32 블루투스 통신](#108_arduino_serial_esp32_comm)   
[109_arduino_motor_remote_control ESP32로 아두이노 모터제어(W,A,S,D사용)](#109_arduino_motor_remote_control)


## 101_motor_no_speed_control

### 📌 코드의 핵심 기능 요약
- 모터를 **1초 동안 회전** 후 **1초 동안 정지**하는 테스트 코드.
- 속도 조절 없이 **단순 ON/OFF 제어**만 수행.
- `digitalWrite()`를 사용하여 모터의 **회전 방향을 HIGH(한쪽 방향)**로 설정.
- 향후 속도 조절(`analogWrite()`) 기능을 추가하면 **PWM 기반 속도 제어**도 가능.

### 1. 핀 정의
```cpp
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11
```
- `motor_A_enable` (핀 12) → 모터 A의 **전원 ON/OFF** 제어
- `motor_B_enable` (핀 13) → 모터 B의 **전원 ON/OFF** 제어
- `motor_A` (핀 10) → 모터 A의 **회전 방향 제어**
- `motor_B` (핀 11) → 모터 B의 **회전 방향 제어**

👉 **L298N 모터 드라이버** 같은 모듈을 사용하여, 모터를 제어하는 방식입니다.

---

### 2. `setup()` 함수
```cpp
void setup() {
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- **모든 핀을 출력 모드(`OUTPUT`)로 설정**하여 전압을 제어할 수 있도록 합니다.

---

### 3. `loop()` 함수
```cpp
void loop() {
  digitalWrite(motor_A_enable, HIGH);
  digitalWrite(motor_B_enable, HIGH);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
  delay(1000);
  
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
  delay(1000);
}
```
📌 **이 코드의 동작 방식**
1. **모터 ON**  
   - `motor_A_enable`과 `motor_B_enable`을 `HIGH`로 설정 → 전원 공급  
   - `motor_A`와 `motor_B`를 `HIGH`로 설정 → **모터 회전**  
   - 1초(`delay(1000)`) 동안 회전

2. **모터 OFF**  
   - `motor_A_enable`과 `motor_B_enable`을 `LOW`로 설정 → **전원 차단**  
   - `motor_A`, `motor_B`는 그대로 HIGH  
   - 1초(`delay(1000)`) 동안 멈춤  

📌 **주석 처리된 부분 (`analogWrite`)**
```cpp
//analogWrite(motor_A, x);
//analogWrite(motor_B, x);
```
- 속도 조절(`PWM`)을 위한 코드가 주석 처리되어 있음 → **단순 ON/OFF 방식으로 동작**.



## 101_motor_variable_speed_control

### **📌 코드의 핵심 기능 요약**
- `analogWrite()`를 이용하여 **모터 속도를 조절**하는 코드.
- 1초 동안 **최대 속도 → 중간 속도 → 정지** 순서로 실행.
- **PWM 신호(0~255 값)를 사용하여 속도를 부드럽게 조절**할 수 있음.

### 1. 핀 정의
```cpp
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11
```
- `motor_A_enable` (핀 12) → 모터 A의 **전원 ON/OFF** 제어
- `motor_B_enable` (핀 13) → 모터 B의 **전원 ON/OFF** 제어
- `motor_A` (핀 10) → 모터 A의 **속도 및 방향 제어 (PWM 출력)**
- `motor_B` (핀 11) → 모터 B의 **속도 및 방향 제어 (PWM 출력)**

👉 **L298N 모터 드라이버** 같은 모듈을 사용하여 모터 속도를 조절합니다.

---

### 2. `setup()` 함수
```cpp
void setup() {
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- **모든 핀을 출력 모드(`OUTPUT`)로 설정**하여 전압을 제어할 수 있도록 합니다.

---

### 3. `loop()` 함수
```cpp
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
```
📌 **이 코드의 동작 방식**
1. **최대 속도로 모터 회전**
   - `motor_A_enable`과 `motor_B_enable`을 `HIGH`로 설정 → 전원 공급
   - `analogWrite(motor_A, 255)`, `analogWrite(motor_B, 255)` → **PWM 신호 255(최대 속도)**
   - 1초(`delay(1000)`) 동안 회전

2. **절반 속도로 모터 회전**
   - `analogWrite(motor_A, 127)`, `analogWrite(motor_B, 127)` → **PWM 신호 127(중간 속도)**
   - 1초(`delay(1000)`) 동안 회전

3. **모터 정지**
   - `analogWrite(motor_A, 0)`, `analogWrite(motor_B, 0)` → **PWM 신호 0 (정지)**
   - 1초(`delay(1000)`) 동안 멈춤

📌 **PWM을 활용한 속도 제어**
- `analogWrite(pin, value)`를 사용하여 **0~255 범위**의 값을 출력
- `255`는 **최대 속도**, `127`은 **절반 속도**, `0`은 **정지**



## 102_IR_test

### **📌 코드의 핵심 기능 요약**
- **IR 센서 2개의 신호를 읽어 시리얼 모니터에 출력하는 코드**.
- `digitalRead()`를 이용하여 센서의 **HIGH(1)/LOW(0) 값을 판별**.
- 시리얼 모니터를 통해 **장애물 감지 여부를 실시간으로 확인 가능**.

### 1. 핀 정의
```cpp
int ir1 = 7;
int ir2 = 8;
```
- `ir1` (핀 7) → **첫 번째 IR 센서 입력 핀**
- `ir2` (핀 8) → **두 번째 IR 센서 입력 핀**

👉 **IR 센서**는 장애물을 감지하면 **HIGH(1) 또는 LOW(0)** 값을 출력하는 방식으로 동작합니다.

---

### 2. `setup()` 함수
```cpp
void setup() {
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  Serial.begin(115200);
}
```
- **IR 센서 핀(`ir1`, `ir2`)을 입력(`INPUT`) 모드로 설정**하여 값을 읽을 수 있도록 설정.
- **시리얼 통신(`Serial.begin(115200)`)을 설정**하여 센서 값을 출력할 수 있도록 설정.

---

### 3. `loop()` 함수
```cpp
void loop() {
 
  int state = digitalRead(ir1);
  Serial.print("ir1 = ");
  Serial.println(state);

  state = digitalRead(ir2);
  Serial.print("ir2 = ");
  Serial.println(state);

  delay(100);
}
```
📌 **이 코드의 동작 방식**
1. **첫 번째 IR 센서(`ir1`)의 상태를 읽음**
   - `digitalRead(ir1)`을 사용하여 HIGH(1) 또는 LOW(0) 값을 가져옴.
   - `Serial.print("ir1 = "); Serial.println(state);`로 시리얼 모니터에 출력.

2. **두 번째 IR 센서(`ir2`)의 상태를 읽음**
   - `digitalRead(ir2)`을 사용하여 HIGH(1) 또는 LOW(0) 값을 가져옴.
   - `Serial.print("ir2 = "); Serial.println(state);`로 시리얼 모니터에 출력.

3. **0.1초(100ms) 대기**
   - `delay(100);`로 센서값을 읽는 주기를 조정.



## 103_ultrasonic_test

### 📌 코드의 핵심 기능 요약
- **초음파 센서를 사용하여 거리 컴퓨팅**을 수행하는 코드.
- `TRIG` 핀을 이용해 **초음파 시험을 전송**하고, `ECHO` 핀을 통해 **반사된 시험을 결과로 수신**.
- `pulseIn()`을 사용하여 시간을 컴퓨팅하고, **거리(제곱은 cm)로 변환**.
- 결과를 **시리얼 모니터에 출력**하여 검사 가능.

---

### 1. 핀 정의
```cpp
#define TRIG A4
#define ECHO A5
```
- `TRIG` (A4 핀) → **초음파 시험 전송 핀**
- `ECHO` (A5 핀) → **반사된 초음파 시험 수신 핀**

👉 **TRIG 핀에서 시험을 지정하고, ECHO 핀에서 수신 후 거리 계산**을 수행.

---

### 2. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}
```
- **시리얼 통신을 115200 baud rate로 설정**하여 거리 값을 검사할 수 있도록 함.
- `TRIG` 핀을 `OUTPUT`으로 설정하여 **초음파 시험을 전송 가능**하게 함.
- `ECHO` 핀을 `INPUT`으로 설정하여 **반사된 시험을 수신 가능**하게 함.

---

### 3. `loop()` 함수
```cpp
void loop()
{
  long duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = duration * 17 / 1000;
  Serial.println(duration);
  Serial.print("DIstance : ");
  Serial.print(distance);
  Serial.println(" Cm");
  delay(1000);
}
```

📌 **이 코드의 동작 방식**
1. **초음파 시험 전송**
   - `TRIG` 핀을 LOW → HIGH (10µs) → LOW 로 전환하여 **초음파 시험을 전송**.

2. **반사된 시험 수신 및 거리 계산**
   - `pulseIn(ECHO, HIGH)`를 사용하여 **반사된 시험과의 시간 계산**.
   - `distance = duration * 17 / 1000;`
     → **음소가 (343m/s)를 이용하여 거리(제곱은 cm)로 변환**.

3. **시리얼 모니터 출력**
   - **`duration` 값을 출력** (반사된 시간).
   - **`distance` 값을 출력** (거리).
   - 1초 (`delay(1000)`) 후 복돌.

## 104_encoder_go_forward

### 📌 코드의 핵심 기능 요약
- **엔코더를 사용하여 양쪽 바퀴의 회전 수를 측정하고 균형을 조정하며 직진하는 코드**.
- `encoder_R` 및 `encoder_L` 인터럽트를 사용하여 **각 바퀴의 회전 속도를 측정**.
- `speed_correct()` 함수를 통해 **두 바퀴의 속도를 자동으로 보정**.
- 3초간 직진한 후 정지하며, **시리얼 모니터를 통해 속도를 출력**.

---

### 1. 핀 정의
```cpp
#define motor_A_1A 5
#define motor_A_1B 6
#define motor_B_1A 10
#define motor_B_1B 9

#define encoder_R 2
#define encoder_L 3
```
- `motor_A_1A`, `motor_A_1B` → **왼쪽 바퀴 모터 제어 핀**
- `motor_B_1A`, `motor_B_1B` → **오른쪽 바퀴 모터 제어 핀**
- `encoder_R` (핀 2) → **오른쪽 바퀴 엔코더 입력 핀**
- `encoder_L` (핀 3) → **왼쪽 바퀴 엔코더 입력 핀**

👉 **모터 드라이버와 엔코더를 이용해 바퀴의 회전 속도를 측정하고 조정**

---

### 2. 모터 및 속도 조정 함수
```cpp
void forward(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, speed_R);
  analogWrite(motor_B_1A, speed_L);
  analogWrite(motor_B_1B, 0);
}

void stopAll() {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, 0);
  analogWrite(motor_B_1A, 0);
  analogWrite(motor_B_1B, 0);
}
```
- `forward(speed_R, speed_L)`: **모터를 정방향으로 회전**시켜 이동.
- `stopAll()`: **모든 모터를 정지**시킴.

---

### 3. 엔코더 인터럽트 및 속도 보정
```cpp
void encoder_R_ISR() {
  encoder_R_cnt++;
}

void encoder_L_ISR() {
  encoder_L_cnt++;
}
```
- 엔코더 값이 변할 때마다 인터럽트를 통해 **회전 수를 증가**시킴.

```cpp
void speed_correct() {
  static int encoder_R_cnt_last = 0;
  static int encoder_L_cnt_last = 0;

  delta_R = encoder_R_cnt - encoder_R_cnt_last;
  delta_L = encoder_L_cnt - encoder_L_cnt_last;

  encoder_R_cnt_last = encoder_R_cnt;
  encoder_L_cnt_last = encoder_L_cnt;

  int diff = delta_R - delta_L;
  int correction_factor = 1;

  if (diff > 0) {
    speed_R -= diff * correction_factor;
    speed_L += diff * correction_factor;
    if (speed_R < 0) speed_R = 0;
    if (speed_L > 255) speed_L = 255;
  } else if (diff < 0) {
    speed_R += -diff * correction_factor;
    speed_L -= -diff * correction_factor;
    if (speed_R > 255) speed_R = 255;
    if (speed_L < 0) speed_L = 0;
  }
}
```
- **양쪽 바퀴의 회전 속도를 비교하여 보정**.
- `delta_R`과 `delta_L`을 비교하여 속도를 조절함.
- `speed_R`과 `speed_L`을 조정하여 **두 바퀴의 속도를 균형 맞춤**.

---

### 4. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);
  pinMode(motor_A_1A, OUTPUT);
  pinMode(motor_A_1B, OUTPUT);
  pinMode(motor_B_1A, OUTPUT);
  pinMode(motor_B_1B, OUTPUT);

  pinMode(encoder_R, INPUT_PULLUP);
  pinMode(encoder_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_R), encoder_R_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder_L), encoder_L_ISR, FALLING);

  start_time = millis();
  stop_flag = false;
}
```
- **엔코더 핀을 `INPUT_PULLUP`으로 설정**하여 풀업 저항 사용.
- `attachInterrupt()`를 사용하여 **엔코더 값 변화를 감지**.
- `start_time`을 기록하여 3초 후 정지하도록 설정.

---

### 5. `loop()` 함수
```cpp
void loop() {
  if (!stop_flag) {
    forward(speed_R, speed_L);
    Serial.print("speed_R: ");
    Serial.println(speed_R);
    Serial.print("speed_L: ");
    Serial.println(speed_L);
    Serial.println("");
  } else {
    stopAll();
  }

  time_curr = millis();
  if (time_curr - time_prev >= 100) {
    time_prev = time_curr;
    speed_correct();
  }

  if (millis() - start_time >= 3000) {
    stop_flag = true;
  }

  delay(50);
}
```
- **모터를 작동시키며 3초 동안 직진**.
- 100ms마다 `speed_correct()`를 호출하여 속도를 보정.
- `millis()`를 사용하여 **3초 후 정지**하도록 설정.

---

## 105_arduino_serial_comm_with_esp32

### 📌 코드의 핵심 기능 요약
- **아두이노와 ESP32 간의 소프트웨어 시리얼 통신을 수행하는 코드**.
- `SoftwareSerial`을 사용하여 **7번(RX), 8번(TX) 핀을 통해 시리얼 데이터 수신**.
- `mySerial.available()`을 이용해 데이터가 수신되었는지 확인하고, **수신된 데이터를 시리얼 모니터에 출력**.

---

### 1. 소프트웨어 시리얼 설정
```cpp
#include <SoftwareSerial.h>

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(7, 8); // RX, TX
```
- `SoftwareSerial` 라이브러리를 포함하여 **소프트웨어 기반 시리얼 통신을 사용**.
- `mySerial(7, 8)` →  **7번 핀을 RX, 8번 핀을 TX로 설정**하여 ESP32와 통신 준비.

---

### 2. `setup()` 함수
```cpp
void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
}
```
- `mySerial.begin(9600);` → **소프트웨어 시리얼 통신 속도를 9600bps로 설정**.
- `Serial.begin(115200);` → **기본 시리얼 모니터 속도를 115200bps로 설정**.
- 프로그램 시작 시 **"SoftwareSerial 데이터 수신 준비 완료" 메시지를 출력**.

---

### 3. `loop()` 함수
```cpp
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
```
- `mySerial.available()`을 사용하여 **데이터가 수신되었는지 확인**.
- 데이터가 수신되면 `mySerial.read()`를 통해 **1바이트(char)씩 읽어들임**.
- 읽은 데이터를 `Serial.print()`를 이용하여 **시리얼 모니터에 출력**.
- 100ms마다 루프를 반복하여 **연속적인 데이터 수신을 처리**.

---

## 105_esp32_serial_comm_with_arduino

### 📌 코드의 핵심 기능 요약
- **ESP32가 하드웨어 시리얼2(Serial2)를 이용하여 아두이노와 통신하는 코드**.
- `Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2)`를 사용하여 **ESP32의 RXD2(16번), TXD2(17번) 핀을 통해 시리얼 통신을 설정**.
- 1초마다 **"hello world"를 기본 시리얼(USB) 출력, "hello 2 world"를 Serial2(UART2)로 출력**.

---

### 1. 핀 정의
```cpp
#define RXD2 16
#define TXD2 17
```
- `RXD2` (16번 핀) → **ESP32의 하드웨어 UART2 RX(수신) 핀**
- `TXD2` (17번 핀) → **ESP32의 하드웨어 UART2 TX(송신) 핀**

👉 **ESP32의 Serial2(UART2) 핀을 사용하여 아두이노와 직접 통신 가능**.

---

### 2. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);  // 시리얼 모니터 시작
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(1000);
}
```
- `Serial.begin(115200);` → **ESP32의 기본 시리얼(USB) 출력 속도를 115200bps로 설정**.
- `Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);` → **ESP32의 UART2(Serial2)를 9600bps로 설정하여 아두이노와 통신 준비**.
- `delay(1000);` → 초기화 지연을 줘서 안정적인 통신이 가능하도록 설정.

---

### 3. `loop()` 함수
```cpp
void loop() {
  Serial.println("hello world");
  Serial2.println("hello 2 world");
  delay(1000);
}
```
- `Serial.println("hello world");` → **ESP32의 기본 시리얼(USB) 모니터에 "hello world" 출력**.
- `Serial2.println("hello 2 world");` → **ESP32의 UART2(Serial2)를 통해 "hello 2 world"를 아두이노에 전송**.
- `delay(1000);` → **1초마다 데이터를 반복 전송**.

---

## 106_esp32_i2c_test

### 📌 코드의 핵심 기능 요약
- **ESP32가 I2C 통신을 이용하여 OLED 디스플레이(SSD1306)를 제어하는 코드**.
- `U8x8lib` 라이브러리를 사용하여 **텍스트를 화면에 출력**.
- `"Hello World!"`와 `"012345678901234567890123456789"` 문자열을 표시하고, **일부 텍스트를 반전(Inverse) 효과로 출력**.

---

### 1. 라이브러리 포함 및 디스플레이 객체 생성
```cpp
#include <Arduino.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
```
- `#include <U8x8lib.h>` → **SSD1306 OLED 디스플레이를 제어하기 위한 라이브러리 포함**.
- `#include <SPI.h>` → **U8x8lib가 SPI 모드도 지원하기 때문에 조건부 포함**.
- `U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);` → **I2C 방식의 128x64 해상도 SSD1306 디스플레이 객체 생성**.

---

### 2. `setup()` 함수
```cpp
void setup(void) {
  u8x8.begin();
  u8x8.setPowerSave(0);
}
```
- `u8x8.begin();` → **디스플레이 초기화 및 I2C 통신 시작**.
- `u8x8.setPowerSave(0);` → **디스플레이 절전 모드 해제**.

---

### 3. `loop()` 함수
```cpp
void loop(void) {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0,1,"Hello World!");
  u8x8.setInverseFont(1);
  u8x8.drawString(0,0,"012345678901234567890123456789");
  u8x8.setInverseFont(0);
  u8x8.refreshDisplay();
  delay(2000);
}
```
- `u8x8.setFont(u8x8_font_chroma48medium8_r);` → **텍스트 출력에 사용할 폰트 설정**.
- `u8x8.drawString(0,1,"Hello World!");` → **좌표 (0,1)에 "Hello World!" 출력**.
- `u8x8.setInverseFont(1);` → **이후 출력될 텍스트를 반전(Inverse) 모드로 설정**.
- `u8x8.drawString(0,0,"012345678901234567890123456789");` → **좌표 (0,0)에 긴 문자열 출력**.
- `u8x8.setInverseFont(0);` → **텍스트 반전 모드를 해제**.
- `u8x8.refreshDisplay();` → **SSD1606/7 같은 일부 디스플레이에서 필요**.
- `delay(2000);` → **2초 동안 화면을 유지한 후 반복**.

---

## 107_robot_drive_with_encoder

### 📌 코드의 핵심 기능 요약
- **ESP32 없이 엔코더를 사용하여 로봇을 직진시키는 코드**.
- `encoder_R` 및 `encoder_L` 인터럽트를 사용하여 **각 바퀴의 회전 수를 측정**.
- `adjustSpeed()` 함수를 통해 **두 바퀴의 속도를 자동으로 보정**.
- 시리얼 입력을 받아 **이동 방향과 속도를 설정**하며, 기본적으로 직진, 후진, 좌회전, 우회전을 수행.

---

### 1. 핀 정의
```cpp
#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

#define encoder_R 2
#define encoder_L 3
```
- `motor_A_enable`, `motor_B_enable` → **모터 속도 조절 핀**
- `motor_A`, `motor_B` → **모터 회전 방향 제어 핀**
- `encoder_R` (핀 2) → **오른쪽 바퀴 엔코더 입력 핀**
- `encoder_L` (핀 3) → **왼쪽 바퀴 엔코더 입력 핀**

👉 **엔코더 값을 기반으로 바퀴 속도를 조정하여 균형 잡힌 직진 주행 가능**.

---

### 2. 모터 제어 함수
```cpp
void forward(int R, int L) {
  analogWrite(motor_A_enable, L);
  analogWrite(motor_B_enable, R);
  digitalWrite(motor_A, HIGH);
  digitalWrite(motor_B, HIGH);
}
```
- `forward(R, L)`: **지정한 속도로 직진**.
- `backward(R, L)`, `turnLeft(R, L)`, `turnRight(R, L)`, `stopAll()`도 유사한 방식으로 구현.

---

### 3. 엔코더 인터럽트 및 속도 보정
```cpp
void encoder_R_ISR(){
  encoder_R_cnt++;
}

void encoder_L_ISR(){
  encoder_L_cnt++;
}
```
- 엔코더 값이 변할 때마다 **회전 수를 증가**.

```cpp
void adjustSpeed() {
  static int last_encoder_R_cnt = 0;
  static int last_encoder_L_cnt = 0;
  
  delta_R = encoder_R_cnt - last_encoder_R_cnt;
  delta_L = encoder_L_cnt - last_encoder_L_cnt;

  last_encoder_R_cnt = encoder_R_cnt;
  last_encoder_L_cnt = encoder_L_cnt;

  int diff = delta_R - delta_L;
  speed_adjust_val = diff * diff_weight;
}
```
- **양쪽 바퀴의 속도를 비교하여 차이를 보정**.
- `diff_weight` 값을 이용하여 **속도 차이에 따른 보정값을 계산**.

---

### 4. `setup()` 함수
```cpp
void setup() {
  Serial.begin(115200);

  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);

  pinMode(encoder_R, INPUT_PULLUP);
  pinMode(encoder_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_R), encoder_R_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder_L), encoder_L_ISR, FALLING);
}
```
- **엔코더 핀을 `INPUT_PULLUP`으로 설정**하여 풀업 저항 사용.
- `attachInterrupt()`를 사용하여 **엔코더 값 변화를 감지**.

---

### 5. `loop()` 함수
```cpp
void loop() {
  time_curr = millis();
  
  if (Serial.available() > 0) {
    inString = Serial.readStringUntil('\n');
    char cmd = inString[0];
    if (cmd == '1') {
      moving_direction = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
      moving_speed = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();

      speed_R = moving_speed + speed_adjust_val;
      speed_L = moving_speed + speed_adjust_val;

      speed_R_turn = speed_R - turn_speed;
      speed_L_turn = speed_R - turn_speed;

      speed_R = constrain(speed_R,0,255);
      speed_L = constrain(speed_L,0,255);
      speed_R_turn = constrain(speed_R_turn,0,255);
      speed_L_turn = constrain(speed_L_turn,0,255);

      if (moving_direction == 1) {
        forward(speed_R, speed_L);
      } else if (moving_direction == 2) {
        backward(speed_R, speed_L);
      } else if (moving_direction == 3) {
        turnLeft(speed_R_turn, speed_L_turn);
      } else if (moving_direction == 4) {
        turnRight(speed_R_turn, speed_L_turn);
      }
    } else if (cmd == '0') {
      diff_weight = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
      encoder_update_interval = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();
      delay_time = inString.substring(inString.indexOf('c') + 1, inString.indexOf('d')).toInt();
      turn_speed = inString.substring(inString.indexOf('d') + 1, inString.indexOf('e')).toInt();
    }
  } else {
    stopAll();
  }

  if (time_curr - time_prev >= encoder_update_interval) {
    time_prev = time_curr;
    adjustSpeed();
  }
  delay(delay_time);
}
```
- **시리얼 입력을 통해 이동 방향과 속도를 설정**.
- `1`을 입력받으면 **방향(moving_direction)과 속도(moving_speed) 값을 분석하여 이동**.
- `0`을 입력받으면 **속도 보정 파라미터(diff_weight, delay_time 등)를 설정**.
- `encoder_update_interval` 마다 `adjustSpeed()`를 호출하여 **속도 차이를 조정**.
- 기본적으로 입력이 없을 경우 `stopAll()`을 실행하여 정지.

---

## 108_arduino_serial_esp32_comm

### 📌 코드의 핵심 기능 요약
- **아두이노와 ESP32 간의 소프트웨어 시리얼 통신을 수행하는 코드**.
- `SoftwareSerial`을 사용하여 **7번(RX), 8번(TX) 핀을 통해 시리얼 데이터 송수신**.
- 수신된 데이터를 **시리얼 모니터에 출력하고, 다시 ESP32로 전송(Echo)**.

---

### 1. 소프트웨어 시리얼 설정
```cpp
#include <SoftwareSerial.h>

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(7, 8); // RX, TX
```
- `SoftwareSerial` 라이브러리를 포함하여 **소프트웨어 기반 시리얼 통신을 사용**.
- `mySerial(7, 8)` → **7번 핀을 RX, 8번 핀을 TX로 설정**하여 ESP32와 통신 준비.

---

### 2. `setup()` 함수
```cpp
void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
}
```
- `mySerial.begin(9600);` → **소프트웨어 시리얼 통신 속도를 9600bps로 설정**.
- `Serial.begin(115200);` → **기본 시리얼 모니터 속도를 115200bps로 설정**.
- 프로그램 시작 시 **"SoftwareSerial 데이터 수신 준비 완료" 메시지를 출력**.

---

### 3. `loop()` 함수
```cpp
void loop() {
  // 소프트웨어 시리얼로 데이터가 수신되었는지 확인
  if (mySerial.available()) {
    String incomingStr = mySerial.readStringUntil('\n');  // 수신한 데이터 읽기
    Serial.print("수신한 데이터: ");
    Serial.println(incomingStr);
    mySerial.println(incomingStr);
  }

  // 100ms 지연
  delay(100);
}
```
- `mySerial.available()`을 사용하여 **데이터가 수신되었는지 확인**.
- 데이터가 수신되면 `mySerial.readStringUntil('\n')`을 통해 **문자열을 읽어옴**.
- 읽은 데이터를 `Serial.print()`를 이용하여 **시리얼 모니터에 출력**.
- 받은 데이터를 다시 `mySerial.println()`을 사용하여 **ESP32로 되돌려 전송(Echo)**.
- 100ms마다 루프를 반복하여 **연속적인 데이터 수신을 처리**.

---

## 109_arduino_motor_remote_control

### 📌 코드의 핵심 기능 요약
- **ESP32에서 수신한 `w, a, s, d` 명령을 기반으로 아두이노에서 모터를 제어하는 코드**.
- `SoftwareSerial`을 이용해 **ESP32로부터 명령을 수신하고, 시리얼 모니터에서도 입력을 받을 수 있음**.
- 전진(`w`), 후진(`s`), 좌회전(`a`), 우회전(`d`), 정지(`space`) 명령을 인식하여 **모터를 제어**.

---

### 1. 핀 정의 및 소프트웨어 시리얼 설정
```cpp
#include <SoftwareSerial.h>

#define motor_A_enable 12
#define motor_B_enable 13
#define motor_A 10
#define motor_B 11

// 소프트웨어 시리얼 핀 설정 (7번: RX, 8번: TX)
SoftwareSerial mySerial(7, 8); // RX, TX
```
- `motor_A_enable`, `motor_B_enable` → **모터 속도 조절 핀**
- `motor_A`, `motor_B` → **모터 회전 방향 제어 핀**
- `mySerial(7, 8)` → **7번 핀을 RX, 8번 핀을 TX로 설정하여 ESP32와 시리얼 통신**

---

### 2. `setup()` 함수
```cpp
void setup() {
  // 소프트웨어 시리얼 시작
  mySerial.begin(9600);
  
  // 기본 시리얼 모니터 시작
  Serial.begin(115200);
  
  Serial.println("SoftwareSerial 데이터 수신 준비 완료");
  pinMode(motor_A, OUTPUT);
  pinMode(motor_B, OUTPUT);
  pinMode(motor_A_enable, OUTPUT);
  pinMode(motor_B_enable, OUTPUT);
}
```
- `mySerial.begin(9600);` → **ESP32와의 시리얼 통신 속도를 9600bps로 설정**.
- `Serial.begin(115200);` → **기본 시리얼 모니터 속도를 115200bps로 설정**.
- 모터 제어를 위해 **모든 관련 핀을 출력 모드로 설정**.

---

### 3. 모터 제어 함수
```cpp
void forward(int R, int L) {
  digitalWrite(motor_A_enable, LOW);
  digitalWrite(motor_B_enable, LOW);
  analogWrite(motor_A, L);
  analogWrite(motor_B, R);
}
```
- `forward(R, L)`: **모터를 정방향으로 회전**하여 전진.
- `backward(R, L)`, `turnLeft(R, L)`, `turnRight(R, L)`, `stopAll()`도 유사한 방식으로 구현됨.

---

### 4. `loop()` 함수 - 명령 수신 및 모터 제어
```cpp
void loop() {
  // 소프트웨어 시리얼로 데이터가 수신되었는지 확인
  if (mySerial.available()) {
    String incomingStr2 = mySerial.readStringUntil('\n');  // 수신한 데이터 읽기
    if(incomingStr2[0] == 'w'){
      Serial.println("forward");
      forward(255, 255);
    }else if(incomingStr2[0] == 's'){
      Serial.println("backward");
      backward(255, 255);
    }else if(incomingStr2[0] == ' '){
      Serial.println("stop");
      stopAll();
    }else if(incomingStr2[0] == 'a'){
      Serial.println("turnleft");
      turnLeft(255, 255);
    }else if(incomingStr2[0] == 'd'){
      Serial.println("turnRight");
      turnRight(255, 255);
    }
  }
```
- **ESP32에서 받은 데이터(`w, a, s, d, space`)를 분석하여 모터 동작 수행**.
- 입력에 따라 **전진, 후진, 좌회전, 우회전, 정지 명령 실행**.

```cpp
  if(Serial.available()){
    String incomingStr = Serial.readStringUntil('\n');
    if(incomingStr[0] == 'w'){
      Serial.println("forward");
      forward(255, 255);
    }else if(incomingStr[0] == 's'){
      Serial.println("backward");
      backward(255, 255);
    }else if(incomingStr[0] == ' '){
      Serial.println("stop");
      stopAll();
    }else if(incomingStr[0] == 'a'){
      Serial.println("turnleft");
      turnLeft(255, 255);
    }else if(incomingStr[0] == 'd'){
      Serial.println("turnRight");
      turnRight(255, 255);
    }
  }
```
- **PC 시리얼 모니터에서 직접 명령 입력 가능**.
- 동일한 명령어(`w, a, s, d, space`)를 사용하여 **모터 제어 가능**.

```cpp
  // 100ms 지연
  delay(100);
}
```
- **명령을 처리한 후 100ms 대기 후 반복 실행**.

---

