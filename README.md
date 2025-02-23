## JDAMR100

### JDAMR100 개요

JDAMR100은 **ROS2 입문자를 위한 아두이노 기반 2바퀴 로봇 키트**입니다. 이 키트는 모듈형 설계를 갖추고 있어 로봇 제어와 네트워킹 기술을 단계별로 학습할 수 있습니다. 사용자는 밸런싱 로봇과 2바퀴 RC카 로봇 두 가지 형태를 구성하여 실험할 수 있으며, 아두이노와 ESP32를 활용하여 다양한 프로그래밍 기법을 익힐 수 있습니다.

JDAMR100은 **센서 활용, 모터 제어, ROS2 연동, IoT 기반 원격 제어** 등의 다양한 주제를 포함하고 있어, 기초부터 고급까지 로봇 개발을 학습하려는 학생과 연구자들에게 적합합니다.

### JDAMR100의 주요 특징

1. 모듈형 로봇

   * **밸런싱 로봇**과 **2바퀴 RC 로봇**의 두 가지 모듈로 변환 가능

   * 학습 목적에 따라 조립 변경 가능

2. 다양한 학습 주제 지원

   * **로봇 밸런싱(Balancing Robot)**: 무게 중심을 유지하며 주행하는 기술 학습

   * **2바퀴 RC카(RC Car) 조종**: 무선으로 조작 가능한 RC카 모드

   * **ROS2 기반 로봇 제어**: ROS2 환경에서 로봇을 운용하는 방법 학습

   * **라인 트레이싱(Line Tracing)**: IR 센서를 이용한 경로 추적 알고리즘 구현

   * **엔코더(Encoder) 활용**: 모터 회전 속도 및 위치 제어

   * **ESP32 기반 블루투스 및 소켓 통신 학습**: WiFi 및 블루투스를 이용한 원격 제어 및 데이터 송수신 학습

3. 하드웨어 및 소프트웨어 요구사항

   * 필수 개발 환경:

     * 아두이노 IDE

     * ESP32 개발 환경 설치

     * ROS2 개발 환경 구축 (선택사항)

   * 하드웨어 준비:

     * JDAMR100 로봇 키트

     * IR 센서, 초음파 센서, 엔코더, OLED 디스플레이 등

4. 학습 자료 및 예제 코드 제공:

   * 단계별로 2바퀴 로봇 및 밸런싱 로봇 학습 예제 코드 제공

   * 로봇 조립 가이드 포함

### 프로젝트 파일 개요

JDAMR100의 예제 코드와 문서는 아래와 같이 구성됩니다:

* README.md: 본 파일로, JDAMR100의 개요 및 주요 기능을 설명합니다.

* doc/assemble.md: JDAMR100의 조립 매뉴얼이 포함되어 있습니다.

* arduino/: 아두이노 및 ESP32에서 실행할 코드들이 포함된 폴더

  * 101_motor_no_speed_control: 모터의 단순 정회전/역회전 테스트

  * 101_motor_variable_speed: analogWrite를 이용한 속도 조절 테스트

  * 102_IR_test: IR 센서 테스트 코드

  * 103_ultrasonic_test: 초음파 센서 테스트 코드

  * 104_encoder_test: 양쪽 바퀴 엔코더 동작 테스트 (사용하지 않음, 삭제 예정)

  * 105_esp32_serial_comm_with_arduino: ESP32에서 정보를 보내고, 아두이노에서 시리얼 모니터로 정보를 받는 코드

  * 106_esp32_i2c_test: OLED 디스플레이 I2C 통신 테스트 코드

  * 107_robot_drive_with_encoder: ESP32 없이 엔코더를 사용하여 직진하는 코드

  * 108_esp32_serial_BT_bridge: 블루투스를 이용한 ESP32와 아두이노 간 통신 테스트 코드

  * 109_arduino_motor_remote_control: ESP32에서 w, a, s, d 키 명령을 받아 아두이노에서 모터를 제어하는 코드

  * 110_socket_serial_bridge: ESP32가 WiFi 소켓으로 받은 신호를 시리얼로 아두이노에 전달하는 코드

  * 111_arduino_robot_remote_control: 109 코드와 중복될 가능성이 있음

  * 112_arduino_ROS_motor_remote_ctrl: ROS 환경에서의 모터 제어 코드 (생략 예정)*

### 사용 준비

1. 아두이노 IDE 설치

    [아두이노 공식 홈페이지](https://www.arduino.cc/en/software)에서 다운로드 후 설치

2. ESP32 보드 매니저 추가 및 설치

    아두이노 IDE 실행 후 파일(File) → 기본 설정(Preferences) 열기

    추가적인 보드 매니저 URLs에 아래 URL 추가:

    ```
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
    ```

    도구(Tools) → 보드(Board) → 보드 매니저(Board Manager)에서 ESP32 검색 후 설치

3. ESP32 보드 설정

    도구(Tools) → 보드(Board) → ESP32 Arduino에서 ESP32 Dev Module 선택

    도구(Tools) → 포트(Port)에서 ESP32가 연결된 포트 선택 (예: COM3 또는 /dev/ttyUSB0)

### 결론

JDAMR100은 **기본적인 로봇 메커니즘부터 고급 ROS2 운용까지 학습할 수 있는 올인원 로봇 키트**입니다.
이 키트를 통해 사용자는 **아두이노 및 ESP32 프로그래밍, IoT 연결, 센서 데이터 활용, PID 제어, 원격 조종** 등의 다양한 로봇 기술을 경험할 수 있습니다.

JDAMR100을 활용하여 다양한 프로젝트를 수행하고, 로봇 및 임베디드 프로그래밍의 기초를 탄탄하게 다져보세요!

보다 자세한 조립 과정 및 사용 방법은 doc/assemble.md에서 확인할 수 있습니다.

