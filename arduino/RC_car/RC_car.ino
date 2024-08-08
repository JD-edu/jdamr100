#define motor_A_1A 5
#define motor_A_1B 6
#define motor_B_1A 10
#define motor_B_1B 9

#define encoder_R 2
#define encoder_L 3

unsigned long time_prev = 0;
unsigned long time_curr = 0;

volatile int encoder_R_cnt = 0;
volatile int encoder_L_cnt = 0;

int moving_direction = 0;
int moving_speed = 0;
int delta_R = 0;
int delta_L = 0;
int speed_R = 0;
int speed_L = 0;

String inString;

void forward(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, speed_R);
  analogWrite(motor_B_1A, speed_L);
  analogWrite(motor_B_1B, 0);
}

void backward(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, speed_R);
  analogWrite(motor_A_1B, 0);
  analogWrite(motor_B_1A, 0);
  analogWrite(motor_B_1B, speed_L);
}

void turnLeft(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, speed_R);
  analogWrite(motor_B_1A, 0);
  analogWrite(motor_B_1B, speed_L);
}

void turnRight(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, speed_R);
  analogWrite(motor_A_1B, 0);
  analogWrite(motor_B_1A, speed_L);
  analogWrite(motor_B_1B, 0);
}

void stopAll() {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, 0);
  analogWrite(motor_B_1A, 0);
  analogWrite(motor_B_1B, 0);
}

void encoder_R_ISR(){
  encoder_R_cnt++;
}

void encoder_L_ISR(){
  encoder_L_cnt++;
}

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
}

void loop() {
  if (Serial.available() > 0) {
    inString = Serial.readStringUntil('\n');
    char cmd = inString[0];
    if (cmd == '1') {
      moving_direction = inString.substring(inString.indexOf('a') + 1, inString.indexOf('b')).toInt();
      moving_speed = inString.substring(inString.indexOf('b') + 1, inString.indexOf('c')).toInt();

      speed_R = moving_speed;
      speed_L = moving_speed;

      if (moving_direction == 1) {
        forward(speed_R, speed_L);
      } else if (moving_direction == 2) {
        backward(speed_R, speed_L);
      } else if (moving_direction == 3) {
        turnLeft(speed_R, speed_L);
      } else if (moving_direction == 4) {
        turnRight(speed_R, speed_L);
      }
    }
  }else {
    stopAll();
  }

  time_curr = millis();
  if (time_curr - time_prev >= 100) {
    time_prev = time_curr;
    speed_correct();
  }
  delay(50);
}
