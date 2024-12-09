#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

//Servo & Gyro Initialization 
MPU6050 mpu;
Servo servoX, servoY;

//Variable Declarations
float Kp = 2.0, Ki = 0.5, Kd = 1.0;
float integralPitch = 0, integralRoll = 0;
float prevErrorPitch = 0, prevErrorRoll = 0;

enum ControlMode { STABILIZE, RECOVERY };
ControlMode currentMode = STABILIZE;

float targetPitch = 0, targetRoll = 0;
const float recoveryThreshold = 15.0;
const float stabilizeThreshold = 5.0;

float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

unsigned long prevTime = 0;

const int filterSize = 10;
float gyroXHistory[filterSize] = {0};
float gyroYHistory[filterSize] = {0};
int filterIndex = 0;

float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
float anglePitch = 0, angleRoll = 0, biasPitch = 0, biasRoll = 0;
float P[2][2] = {{1, 0}, {0, 1}};

bool userSetpoint = false; 

int redPin = 5;
int greenPin = 6;
int bluePin = 7;
int currentRed = 0, currentGreen = 255, currentBlue = 0;
int targetRed = 0, targetGreen = 255, targetBlue = 0;
float transitionSpeed = 0.05;
unsigned long lastBlinkTime = 0;
bool blinkState = false;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
  if (!mpu.testConnection()) {
    setErrorLED(1); 
    while (1);
  }

  servoX.attach(9);
  servoY.attach(10);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  runChecks();
  homeAxes();
  prevTime = millis();
}

void loop() {
  float pitch, roll;
  filterOrientation(&pitch, &roll);

  float errorPitch = targetPitch - pitch;
  float errorRoll = targetRoll - roll;

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;

  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'K') {
      Serial.println("Enter Kp, Ki, Kd values: ");
      while (Serial.available() == 0) {}
      Kp = Serial.parseFloat();
      Ki = Serial.parseFloat();
      Kd = Serial.parseFloat();
      Serial.print("New Kp: "); Serial.println(Kp);
      Serial.print("New Ki: "); Serial.println(Ki);
      Serial.print("New Kd: "); Serial.println(Kd);
    } else if (command == 'T') {
      Serial.println("Enter Target Pitch and Roll values (degrees): ");
      while (Serial.available() == 0) {}
      targetPitch = Serial.parseFloat();
      targetRoll = Serial.parseFloat();
      userSetpoint = true;
      Serial.print("New Target Pitch: "); Serial.println(targetPitch);
      Serial.print("New Target Roll: "); Serial.println(targetRoll);
    }
  }

  if (!userSetpoint) {
    targetPitch = 0;
    targetRoll = 0;
  }

  if (currentMode == STABILIZE && (abs(errorPitch) > recoveryThreshold || abs(errorRoll) > recoveryThreshold)) {
    switchMode(RECOVERY);
  } else if (currentMode == RECOVERY && (abs(errorPitch) < stabilizeThreshold && abs(errorRoll) < stabilizeThreshold)) {
    switchMode(STABILIZE);
  }

  float outputPitch = computePID(errorPitch, deltaTime, &integralPitch, &prevErrorPitch);
  float outputRoll = computePID(errorRoll, deltaTime, &integralRoll, &prevErrorRoll);

  int servoXPos = constrain(90 + outputPitch, 0, 180);
  int servoYPos = constrain(90 + outputRoll, 0, 180);

  servoX.write(servoXPos);
  servoY.write(servoYPos);

  updateLED(abs(errorPitch), abs(errorRoll));

  prevTime = currentTime;
  delay(10);
}

void runChecks() {
  Serial.println("Running Pre-flight Check");

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  if (ax == 0 && ay == 0 && az == 0) {
    setErrorLED(2); 
    while (1);
  }

  if (gx == 0 && gy == 0 && gz == 0) {
    setErrorLED(3); 
    while (1);
  }

  Serial.println("Pre-flight check passed!");
}

void homeAxes() {
  servoX.write(0);
  delay(500);
  servoX.write(180);
  delay(500);
  servoX.write(90);

  servoY.write(0);
  delay(500);
  servoY.write(180);
  delay(500);
  servoY.write(90);

  if (!calibrateGyro()) {
    setErrorLED(4); 
    while (1);
  }
}

bool calibrateGyro() {
  long sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    int16_t gyroX, gyroY, gyroZ;
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    delay(10);
  }

  gyroXOffset = sumX / samples;
  gyroYOffset = sumY / samples;
  gyroZOffset = sumZ / samples;

  return true;
}

float computePID(float error, float deltaTime, float *integral, float *prevError) {
  *integral += error * deltaTime;
  *integral = constrain(*integral, -1000, 1000);  

  float derivative = (error - *prevError) / deltaTime;
  float output = Kp * error + Ki * *integral + Kd * derivative;
  *prevError = error;
  return output;
}

void filterOrientation(float* pitch, float* roll) {
  int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

  gyroX = (gyroX - gyroXOffset) / 131.0;
  gyroY = (gyroY - gyroYOffset) / 131.0;

  gyroXHistory[filterIndex] = gyroX;
  gyroYHistory[filterIndex] = gyroY;
  filterIndex = (filterIndex + 1) % filterSize;

  gyroX = getFilteredValue(gyroXHistory, filterSize);
  gyroY = getFilteredValue(gyroYHistory, filterSize);

  float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
  float accelRoll = atan2(-accelX, accelZ) * 180 / PI;

  kalmanFilter(accelPitch, gyroX, &anglePitch, &biasPitch);
  kalmanFilter(accelRoll, gyroY, &angleRoll, &biasRoll);

  *pitch = anglePitch;
  *roll = angleRoll;
}

void kalmanFilter(float accelAngle, float gyroRate, float* angle, float* bias) {
  float dt = (millis() - prevTime) / 1000.0;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float y = accelAngle - *angle;
  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  *angle += K[0] * y;
  *bias += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}

float getFilteredValue(float* history, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += history[i];
  }
  return sum / size;
}

void switchMode(ControlMode mode) {
  currentMode = mode;
}

void updateLED(float pitchError, float rollError) {
  float error = max(pitchError, rollError);

  if (error > 20) { 
    targetRed = 255;
    targetGreen = 0;
    targetBlue = 0;
  } else if (error > 10) { 
    targetRed = 255;
    targetGreen = map(error, 10, 20, 0, 255); 
    targetBlue = 0;
  } else if (error > 5) {
    targetRed = 0;
    targetGreen = map(error, 5, 10, 255, 255); 
    targetBlue = 0;
  } else {
    targetRed = 0;
    targetGreen = 255;
    targetBlue = 0;
  }

  currentRed = LERP(currentRed, targetRed, transitionSpeed);
  currentGreen = LERP(currentGreen, targetGreen, transitionSpeed);
  currentBlue = LERP(currentBlue, targetBlue, transitionSpeed);

  analogWrite(redPin, currentRed);
  analogWrite(greenPin, currentGreen);
  analogWrite(bluePin, currentBlue);
}

int LERP(int start, int end, float t) {
  return start + (end - start) * t;
}

void setErrorLED(int errorCode) {
  unsigned long currentMillis = millis();

  if (currentMillis - lastBlinkTime > 500) {
    lastBlinkTime = currentMillis;
    blinkState = !blinkState;

    if (blinkState) {
      switch (errorCode) {
        case 1:
          analogWrite(redPin, 255); 
          analogWrite(greenPin, 0); 
          analogWrite(bluePin, 0); 
          break;
        case 2:
          analogWrite(redPin, 255); 
          analogWrite(greenPin, 165); 
          analogWrite(bluePin, 0); 
          break;
        case 3:
          analogWrite(redPin, 255); 
          analogWrite(greenPin, 165); 
          analogWrite(bluePin, 0); 
          break;
        case 4:
          analogWrite(redPin, 255); 
          analogWrite(greenPin, 0); 
          analogWrite(bluePin, 255); 
          break;
        default:
          analogWrite(redPin, 0); 
          analogWrite(greenPin, 0); 
          analogWrite(bluePin, 255); 
          break;
      }
    } else {
      analogWrite(redPin, 0); 
      analogWrite(greenPin, 0); 
      analogWrite(bluePin, 0); 
    }
  }
}
