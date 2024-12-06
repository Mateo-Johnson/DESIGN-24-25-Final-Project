#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servoX, servoY;

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

unsigned long lastBlinkTime = 0;
bool blinkState = false;

// Pin definitions for RGB LED
const int redPin = 5;
const int greenPin = 6;
const int bluePin = 9;

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

  // Set RGB LED pins as OUTPUT
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  runPreFlightCheck();
  runHomingSequence();
  prevTime = millis();
}

void loop() {
  float pitch, roll;
  getFilteredOrientation(&pitch, &roll);

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
    }
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

  updateRGBLED(abs(errorPitch), abs(errorRoll));

  prevTime = currentTime;
  delay(10);
}

void runPreFlightCheck() {
  Serial.println("Running Pre-flight Check...");

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

void runHomingSequence() {
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
  float derivative = (error - *prevError) / deltaTime;
  float output = Kp * error + Ki * *integral + Kd * derivative;
  *prevError = error;
  return output;
}

void getFilteredOrientation(float* pitch, float* roll) {
  float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
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

void updateRGBLED(float pitchError, float rollError) {
  float error = max(pitchError, rollError);
  int red = 0, green = 255, blue = 0;

  if (error > 20) {
    red = 255;
    green = 0;
  } else if (error > 10) {
    red = 255;
    green = 255;
  } else if (error > 5) {
    green = 255;
  }

  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
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
