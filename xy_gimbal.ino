#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>  

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

Servo yawServo;
Servo rollServo; 

int const INTERRUPT_PIN = 2;  
bool blinkState;

bool DMPReady = false; 
uint8_t MPUIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint8_t FIFOBuffer[64]; 

Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

volatile bool MPUInterrupt = false;     
void DMPDataReady() {
  MPUInterrupt = true;
}

float yawSetpoint = 0;
float yawInput = 0;
float yawOutput = 0;
float yawPrevError = 0;
float yawIntegral = 0;

float Kp_yaw = 0.7;         
float Ki_yaw = 0.02;        
float Kd_yaw = 0.8;

const float yawIntegralLimit = 20;  
float yawFilteredOutput = 90;      
const float yawFilterAlpha = 0.3;   

float rollSetpoint = 0;
float rollInput = 0;
float rollOutput = 0;
float rollPrevError = 0;
float rollIntegral = 0;

float Kp_roll = 0.7;         
float Ki_roll = 0.02;        
float Kd_roll = 0.8;

const float rollIntegralLimit = 20;  
float rollFilteredOutput = 90;       
const float rollFilterAlpha = 0.3;  

const float deadband = 0.5;

const float smoothingFactor = 0.1;  // Smoothing factor for PID output (adjust this)

float applySmoothing(float previousValue, float newValue, float factor) {
  return (previousValue * (1 - factor)) + (newValue * factor);
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); 
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read());
  while (!Serial.available());                 
  while (Serial.available() && Serial.read()); 

  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  
    mpu.CalibrateGyro(6);
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); 
  } else {
    Serial.print(F("DMP Initialization failed (code ")); 
    Serial.println(devStatus);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  yawServo.attach(10);  
  rollServo.attach(3);  

  yawServo.write(90);  
  rollServo.write(90); 
}

void loop() {
  if (!DMPReady) return; 
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
      yawInput = ypr[0] * 180 / M_PI; 
      float yawError = yawSetpoint - yawInput;
      if (abs(yawError) < deadband) yawError = 0;

      if (abs(yawError) < 10) {
        yawIntegral += yawError;
        yawIntegral = constrain(yawIntegral, -yawIntegralLimit, yawIntegralLimit);
      }
      float yawDerivative = yawError - yawPrevError;
      yawOutput = Kp_yaw * yawError + Ki_yaw * yawIntegral + Kd_yaw * yawDerivative;
      yawPrevError = yawError;

      yawFilteredOutput = applySmoothing(yawFilteredOutput, yawOutput, smoothingFactor);
      
      int yawServoPosition = constrain(map(yawFilteredOutput, -90, 90, 0, 180), 0, 180);
      yawServo.write(yawServoPosition);

      rollInput = ypr[2] * 180 / M_PI;  
      float rollError = - (rollSetpoint - rollInput); 
      if (abs(rollError) < deadband) rollError = 0;

      if (abs(rollError) < 10) {
        rollIntegral += rollError;
        rollIntegral = constrain(rollIntegral, -rollIntegralLimit, rollIntegralLimit);
      }
      float rollDerivative = rollError - rollPrevError;
      rollOutput = Kp_roll * rollError + Ki_roll * rollIntegral + Kd_roll * rollDerivative;
      rollPrevError = rollError;

      rollFilteredOutput = applySmoothing(rollFilteredOutput, rollOutput, smoothingFactor);
      
      int rollServoPosition = constrain(map(rollFilteredOutput, -90, 90, 0, 180), 0, 180);
      rollServo.write(rollServoPosition);

      Serial.print("Yaw:\t");
      Serial.print(yawInput);
      Serial.print("\tYaw Servo:\t");
      Serial.print(yawServoPosition);

      Serial.print("\tRoll:\t");
      Serial.print(rollInput);
      Serial.print("\tRoll Servo:\t");
      Serial.println(rollServoPosition);
    #endif

    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }
}
