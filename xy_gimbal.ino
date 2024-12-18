#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

MPU6050 mpu;

enum ControlMode {
  NORMAL,
  SPORT,
  CINEMA
};

struct PositionState {
  float yaw;
  float roll;
};

const int MAX_PRESETS = 5;
struct {
  PositionState positions[MAX_PRESETS];
  bool used[MAX_PRESETS];
} presets;

struct Config {
  PositionState homePosition;
  float maxSpeed; 
  ControlMode mode;
} config;

struct PIDSet {
  float p;
  float i;
  float d;
};

const PIDSet NORMAL_PID = {0.4, 0.01, 0.3};
const PIDSet SPORT_PID = {0.7, 0.02, 0.5};
const PIDSet CINEMA_PID = {0.2, 0.005, 0.2};

float p_yaw, i_yaw, d_yaw;
float p_roll, i_roll, d_roll;

Servo yawServo;
Servo rollServo;
bool emergencyStop = false;
String inputString = "";
bool stringComplete = false;

float yawSetpoint = 0;
float rollSetpoint = 0;
float targetYaw = 0;
float targetRoll = 0;
float currentSpeed = 10.0;  
unsigned long lastMoveTime = 0;

bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool MPUInterrupt = false;

const int FILTER_SIZE = 5;
float yawBuffer[FILTER_SIZE] = {0};
float rollBuffer[FILTER_SIZE] = {0};
int bufferIndex = 0;

float yawInput, yawOutput, yawPrevError = 0, yawIntegral = 0;
float rollInput, rollOutput, rollPrevError = 0, rollIntegral = 0;
const float yawIntegralLimit = 15;
const float rollIntegralLimit = 15;
float yawFilteredOutput = 90;
float rollFilteredOutput = 90;
const float filterAlpha = 0.15;
const float deadband = 0.8;

void DMPDataReady() {
  MPUInterrupt = true;
}

void updatePIDParameters() {
  PIDSet currentSet;
  switch (config.mode) {
    case SPORT:
      currentSet = SPORT_PID;
      break;
    case CINEMA:
      currentSet = CINEMA_PID;
      break;
    default:
      currentSet = NORMAL_PID;
  }
  
  p_yaw = currentSet.p;
  i_yaw = currentSet.i;
  d_yaw = currentSet.d;
  p_roll = currentSet.p;
  i_roll = currentSet.i;
  d_roll = currentSet.d;
}

void processCommands() {
  if (!stringComplete) return;
  
  inputString.toUpperCase();
  
  if (inputString.startsWith("STOP")) {
    emergencyStop = true;
    yawServo.write(90);
    rollServo.write(90);
    Serial.println("EMERGENCY STOP ACTIVATED");
  }
  else if (inputString.startsWith("RESUME")) {
    emergencyStop = false;
    Serial.println("System resumed");
  }
  else if (inputString.startsWith("Y:")) {
    if (!emergencyStop) {
      float newYaw = inputString.substring(2).toFloat();
      if (newYaw >= -90 && newYaw <= 90) {
        targetYaw = newYaw;
        Serial.print("Moving yaw to: ");
        Serial.println(newYaw);
      }
    }
  }
  else if (inputString.startsWith("R:")) {
    if (!emergencyStop) {
      float newRoll = inputString.substring(2).toFloat();
      if (newRoll >= -90 && newRoll <= 90) {
        targetRoll = newRoll;
        Serial.print("Moving roll to: ");
        Serial.println(newRoll);
      }
    }
  }
  else if (inputString.startsWith("B:")) {
    if (!emergencyStop) {
      int commaIndex = inputString.indexOf(',');
      if (commaIndex != -1) {
        float newYaw = inputString.substring(2, commaIndex).toFloat();
        float newRoll = inputString.substring(commaIndex + 1).toFloat();
        if (newYaw >= -90 && newYaw <= 90 && newRoll >= -90 && newRoll <= 90) {
          targetYaw = newYaw;
          targetRoll = newRoll;
          Serial.println("Moving to new position");
        }
      }
    }
  }
  else if (inputString.startsWith("SPEED:")) {
    float newSpeed = inputString.substring(6).toFloat();
    if (newSpeed > 0 && newSpeed <= 100) {
      config.maxSpeed = newSpeed;
      Serial.print("Speed set to: ");
      Serial.println(newSpeed);
    }
  }
  else if (inputString.startsWith("MODE:")) {
    String mode = inputString.substring(5);
    if (mode == "NORMAL") {
      config.mode = NORMAL;
      updatePIDParameters();
      Serial.println("Normal mode activated");
    }
    else if (mode == "SPORT") {
      config.mode = SPORT;
      updatePIDParameters();
      Serial.println("Sport mode activated");
    }
    else if (mode == "CINEMA") {
      config.mode = CINEMA;
      updatePIDParameters();
      Serial.println("Cinema mode activated");
    }
  }
  else if (inputString.startsWith("HOME")) {
    if (!emergencyStop) {
      targetYaw = config.homePosition.yaw;
      targetRoll = config.homePosition.roll;
      Serial.println("Moving to home position");
    }
  }
  else if (inputString.startsWith("SETHOME")) {
    config.homePosition.yaw = yawInput;
    config.homePosition.roll = rollInput;
    Serial.println("Current position set as home");
  }
  else if (inputString.startsWith("STATUS")) {
    Serial.println("\n--- System Status ---");
    Serial.print("Mode: ");
    Serial.println(config.mode == NORMAL ? "Normal" : (config.mode == SPORT ? "Sport" : "Cinema"));
    Serial.print("Current Position - Yaw: ");
    Serial.print(yawInput);
    Serial.print(" Roll: ");
    Serial.println(rollInput);
    Serial.print("Target Position - Yaw: ");
    Serial.print(targetYaw);
    Serial.print(" Roll: ");
    Serial.println(targetRoll);
    Serial.print("Speed: ");
    Serial.println(config.maxSpeed);
    Serial.print("Emergency Stop: ");
    Serial.println(emergencyStop ? "Active" : "Inactive");
  }
  else if (inputString.startsWith("RY:")) {
    if (!emergencyStop) {
      float relativeYaw = inputString.substring(3).toFloat();
      if (abs(relativeYaw) <= 90) {
        targetYaw = yawInput + relativeYaw;
        targetYaw = constrain(targetYaw, -90, 90);
        Serial.print("Moving yaw by: ");
        Serial.print(relativeYaw);
        Serial.print(" to: ");
        Serial.println(targetYaw);
      }
    }
  }
  else if (inputString.startsWith("RR:")) {
    if (!emergencyStop) {
      float relativeRoll = inputString.substring(3).toFloat();
      if (abs(relativeRoll) <= 90) {
        targetRoll = rollInput + relativeRoll;
        targetRoll = constrain(targetRoll, -90, 90);
        Serial.print("Moving roll by: ");
        Serial.print(relativeRoll);
        Serial.print(" to: ");
        Serial.println(targetRoll);
      }
    }
  }
  else if (inputString.startsWith("RB:")) {
    if (!emergencyStop) {
      int commaIndex = inputString.indexOf(',');
      if (commaIndex != -1) {
        float relativeYaw = inputString.substring(3, commaIndex).toFloat();
        float relativeRoll = inputString.substring(commaIndex + 1).toFloat();
        if (abs(relativeYaw) <= 90 && abs(relativeRoll) <= 90) {
          targetYaw = constrain(yawInput + relativeYaw, -90, 90);
          targetRoll = constrain(rollInput + relativeRoll, -90, 90);
          Serial.println("Moving relatively to new position");
        }
      }
    }
  }
  else if (inputString.startsWith("HELP")) {
    Serial.println("\nAvailable Commands:");
    Serial.println("STOP - Emergency stop");
    Serial.println("RESUME - Resume operation");
    Serial.println("Y:angle - Set yaw angle (-90 to 90)");
    Serial.println("R:angle - Set roll angle (-90 to 90)");
    Serial.println("B:yaw,roll - Set both angles");
    Serial.println("SPEED:value - Set movement speed (1-100)");
    Serial.println("MODE:NORMAL|SPORT|CINEMA - Set control mode");
    Serial.println("HOME - Move to home position");
    Serial.println("SETHOME - Set current position as home");
    Serial.println("STATUS - Display system status");
    Serial.println("RY:angle - Move yaw relative to current position");
    Serial.println("RR:angle - Move roll relative to current position");
    Serial.println("RB:yaw,roll - Move both axes relative to current position");
    Serial.println("HELP - Show this message");
  }

  inputString = "";
  stringComplete = false;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  
  Serial.begin(115200);
  while (!Serial);
  
  mpu.initialize();
  pinMode(2, INPUT);
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while(1);
  }
  
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  
  yawServo.attach(10);
  rollServo.attach(3);
  yawServo.write(90);
  rollServo.write(90);
  
  updatePIDParameters();
  
  Serial.println("System ready. Type HELP for commands.");
}

void updateSetpoints() {
  unsigned long currentTime = millis();
  float elapsed = (currentTime - lastMoveTime) / 1000.0;
  lastMoveTime = currentTime;
  
  float maxDelta = config.maxSpeed * elapsed;
  
  if (abs(targetYaw - yawSetpoint) > maxDelta) {
    yawSetpoint += (targetYaw > yawSetpoint) ? maxDelta : -maxDelta;
  } else {
    yawSetpoint = targetYaw;
  }
  
  if (abs(targetRoll - rollSetpoint) > maxDelta) {
    rollSetpoint += (targetRoll > rollSetpoint) ? maxDelta : -maxDelta;
  } else {
    rollSetpoint = targetRoll;
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    inputString += c;
    if (c == '\n') {
      stringComplete = true;
    }
  }
  
  processCommands();
  
  if (!DMPReady || emergencyStop) return;
  
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    yawBuffer[bufferIndex] = ypr[0] * 180 / M_PI;
    rollBuffer[bufferIndex] = ypr[2] * 180 / M_PI;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    
    yawInput = 0;
    rollInput = 0;
    for(int i = 0; i < FILTER_SIZE; i++) {
      yawInput += yawBuffer[i];
      rollInput += rollBuffer[i];
    }
    yawInput /= FILTER_SIZE;
    rollInput /= FILTER_SIZE;
    
    updateSetpoints();
    
    float yawError = yawSetpoint - yawInput;
    if (abs(yawError) < deadband) yawError = 0;
    
    if (abs(yawError) < 10) {
      yawIntegral += yawError;
      yawIntegral = constrain(yawIntegral, -yawIntegralLimit, yawIntegralLimit);
    }
    
    float yawDerivative = yawError - yawPrevError;
    yawOutput = p_yaw * yawError + i_yaw * yawIntegral + d_yaw * yawDerivative;
    yawPrevError = yawError;
    
    float rollError = -(rollSetpoint - rollInput);
    if (abs(rollError) < deadband) rollError = 0;
    
    if (abs(rollError) < 10) {
      rollIntegral += rollError;
      rollIntegral = constrain(rollIntegral, -rollIntegralLimit, rollIntegralLimit);
    }
    
    float rollDerivative = rollError - rollPrevError;
    rollOutput = p_roll * rollError + i_roll * rollIntegral + d_roll * rollDerivative;
    rollPrevError = rollError;
    
    yawFilteredOutput = (filterAlpha * yawOutput) + (1 - filterAlpha) * yawFilteredOutput;
    rollFilteredOutput = (filterAlpha * rollOutput) + (1 - filterAlpha) * rollFilteredOutput;
    
    int yawServoPosition = constrain(map(yawFilteredOutput, -90, 90, 0, 180), 0, 180);
    int rollServoPosition = constrain(map(rollFilteredOutput, -90, 90, 0, 180), 0, 180);
    
    yawServo.write(yawServoPosition);
    rollServo.write(rollServoPosition);
  }
}
