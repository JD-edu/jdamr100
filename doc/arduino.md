# JDAMR100 아두이노 코드

[101_motor_no_speed_control 모터 정/역회전 테스트](#101_motor_no_speed_control)   
[101_motor_variable_speed_control 모터 속도 조절 테스트](#101_motor_variable_speed_control)   
[102_ir_test IR 센서 테스트](#102_ir_test)

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



#