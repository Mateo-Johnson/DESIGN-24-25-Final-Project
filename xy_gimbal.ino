#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

MPU6050 mpu;
Servo servoX, servoY;
Adafruit_NeoPixel led(1, 6, NEO_GRB + NEO_KHZ800); 

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

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
  if (!mpu.testConnection()) while (1);

  servoX.attach(9);
  servoY.attach(10);
  led.begin();
  led.show();

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
    Serial.println("Accelerometer Test Failed");
    while (1);
  }

  if (gx == 0 && gy == 0 && gz == 0) {
    Serial.println("Gyroscope Test Failed");
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

  calibrateGyro();
}

void calibrateGyro() {
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

  static float gyroPitch = 0;
  static float gyroRoll = 0;

  float dt = (millis() - prevTime) / 1000.0;

  gyroPitch += gyroX * dt;
  gyroRoll += gyroY * dt;

  const float alpha = 0.98;
  *pitch = alpha * gyroPitch + (1 - alpha) * accelPitch;
  *roll = alpha * gyroRoll + (1 - alpha) * accelRoll;
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

  led.setPixelColor(0, led.Color(red, green, blue));
  led.show();
}
