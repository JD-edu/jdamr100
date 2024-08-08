#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


double setpoint= 180;
double Kp = 10; 
double Ki = 95; 
double Kd = 0.99;

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); 

    if (devStatus == 0)
    {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
        
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode (6, OUTPUT);
    pinMode (9, OUTPUT);
    pinMode (10, OUTPUT);
    pinMode (11, OUTPUT);

    analogWrite(6,LOW);
    analogWrite(9,LOW);
    analogWrite(10,LOW);
    analogWrite(11,LOW);
}

 

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize)
{
        pid.Compute();   
      
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
               
        if (input>150 && input<200){//로봇이 범위 내에서 기울어지는 경우
        if (output>0) //앞으로
        Forward(); //전진
        else if (output<0) //뒤로
        Reverse(); //후진
        }
        else //로봇이 기울어지지 않은 경우
        Stop(); //모터 정지
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    }
    else if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

        input = ypr[2] * 180/M_PI + 180;
   }
}

void Forward() //전진
{
    analogWrite(6,output);
    analogWrite(9,0);
    analogWrite(10,output);
    analogWrite(11,0);
    Serial.print("F"); 
}

void Reverse() //후진
{
    analogWrite(6,0);
    analogWrite(9,output*-1);
    analogWrite(10,0);
    analogWrite(11,output*-1); 
    Serial.print("R");
}

void Stop() //정지
{
    analogWrite(6,0);
    analogWrite(9,0);
    analogWrite(10,0);
    analogWrite(11,0); 
    Serial.print("S");
}

