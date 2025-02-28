/*
 * Diagnostic Function for Gyroscopic Rocket Stabilization
 * 
 * Hardware requirements:
 * - Arduino board (compatible with SAMD21)
 * - MPU6050 or similar 6-axis gyroscope/accelerometer
 * - 2x micro servo motors for thrust vector control
 * - Optional: RGB LED for status indication
 */

#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

// Pin definitions
#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10
#define LED_R_PIN 5
#define LED_G_PIN 6
#define LED_B_PIN 7

// System constants
#define NUM_SAMPLES 30
#define RESPONSE_TEST_ANGLE 15.0  // Degrees
#define TARGET_RESPONSE_TIME 16.0 // ms

// Global objects
MPU6050 mpu;
Servo servoX;
Servo servoY;

// Global variables
float gyroData[3];       // x, y, z gyro readings
float accelData[3];      // x, y, z accel readings
float currentAngles[3];  // pitch, roll, yaw
float kalmanAngles[3];   // filtered angles

// Simple Kalman filter variables
float kalmanGain = 0.75;
float previousEstimate[3] = {0, 0, 0};
float estimateUncertainty = 1.0;
float processNoise = 0.01;
float measurementNoise = 0.1;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println(F("=== Gyroscopic Stabilization System Diagnostics ==="));
  
  // Initialize LED pins
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  
  // Set LED to blue during initialization
  setLED(0, 0, 255);
  
  // Initialize servos
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoX.write(90); // Center position
  servoY.write(90); // Center position
  delay(500);
  
  // Initialize I2C and MPU6050
  Wire.begin();
  Serial.println(F("Initializing MPU6050..."));
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println(F("Could not find a valid MPU6050 sensor, check wiring!"));
    setLED(255, 0, 0); // Red indicates error
    delay(500);
  }
  
  // Calibrate gyroscope
  Serial.println(F("Calibrating gyroscope, please keep device still..."));
  mpu.calibrateGyro();
  
  Serial.println(F("Initialization complete!"));
  setLED(0, 255, 0); // Green indicates ready
  delay(1000);
}

void loop() {
  Serial.println(F("\n=== Starting Diagnostic Tests ==="));
  
  // Test 1: Sensor readings
  testSensors();
  
  // Test 2: Response time
  testResponseTime();
  
  // Test 3: Stability
  testStability();
  
  // Report overall status
  reportOverallStatus();
  
  // Wait before running diagnostics again
  Serial.println(F("Diagnostics complete. Restarting in 10 seconds..."));
  delay(10000);
}

void testSensors() {
  Serial.println(F("\n--- Test 1: Sensor Readings ---"));
  setLED(255, 255, 0); // Yellow during sensor test
  
  // Get initial readings
  float gyroSum[3] = {0, 0, 0};
  float accelSum[3] = {0, 0, 0};
  float gyroVariance[3] = {0, 0, 0};
  float accelVariance[3] = {0, 0, 0};
  
  // Take multiple samples
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Vector normGyro = mpu.readNormalizeGyro();
    Vector normAccel = mpu.readNormalizeAccel();
    
    // Store current reading
    gyroData[0] = normGyro.XAxis;
    gyroData[1] = normGyro.YAxis;
    gyroData[2] = normGyro.ZAxis;
    
    accelData[0] = normAccel.XAxis;
    accelData[1] = normAccel.YAxis;
    accelData[2] = normAccel.ZAxis;
    
    // Accumulate for average
    for (int j = 0; j < 3; j++) {
      gyroSum[j] += gyroData[j];
      accelSum[j] += accelData[j];
    }
    
    delay(10);
  }
  
  // Calculate averages
  float gyroAvg[3], accelAvg[3];
  for (int i = 0; i < 3; i++) {
    gyroAvg[i] = gyroSum[i] / NUM_SAMPLES;
    accelAvg[i] = accelSum[i] / NUM_SAMPLES;
  }
  
  // Calculate variances (second pass through data)
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Vector normGyro = mpu.readNormalizeGyro();
    Vector normAccel = mpu.readNormalizeAccel();
    
    for (int j = 0; j < 3; j++) {
      gyroVariance[j] += pow(normGyro.XAxis - gyroAvg[0], 2) / NUM_SAMPLES;
      accelVariance[j] += pow(normAccel.XAxis - accelAvg[0], 2) / NUM_SAMPLES;
    }
    
    delay(10);
  }
  
  // Report results
  Serial.println(F("Gyroscope readings (deg/s):"));
  Serial.print(F("  X-axis: ")); Serial.print(gyroAvg[0]); 
  Serial.print(F(" (±")); Serial.print(sqrt(gyroVariance[0])); Serial.println(F(")"));
  
  Serial.print(F("  Y-axis: ")); Serial.print(gyroAvg[1]); 
  Serial.print(F(" (±")); Serial.print(sqrt(gyroVariance[1])); Serial.println(F(")"));
  
  Serial.print(F("  Z-axis: ")); Serial.print(gyroAvg[2]); 
  Serial.print(F(" (±")); Serial.print(sqrt(gyroVariance[2])); Serial.println(F(")"));
  
  Serial.println(F("Accelerometer readings (g):"));
  Serial.print(F("  X-axis: ")); Serial.print(accelAvg[0]); 
  Serial.print(F(" (±")); Serial.print(sqrt(accelVariance[0])); Serial.println(F(")"));
  
  Serial.print(F("  Y-axis: ")); Serial.print(accelAvg[1]); 
  Serial.print(F(" (±")); Serial.print(sqrt(accelVariance[1])); Serial.println(F(")"));
  
  Serial.print(F("  Z-axis: ")); Serial.print(accelAvg[2]); 
  Serial.print(F(" (±")); Serial.print(sqrt(accelVariance[2])); Serial.println(F(")"));
  
  // Check if noise levels are acceptable
  bool sensorsOK = true;
  for (int i = 0; i < 3; i++) {
    if (sqrt(gyroVariance[i]) > 0.5 || sqrt(accelVariance[i]) > 0.05) {
      sensorsOK = false;
    }
  }
  
  if (sensorsOK) {
    Serial.println(F("RESULT: Sensor readings are stable."));
  } else {
    Serial.println(F("RESULT: Excessive sensor noise detected. Check mounting and connections."));
  }
}

void testResponseTime() {
  Serial.println(F("\n--- Test 2: Response Time Test ---"));
  setLED(255, 0, 255); // Purple during response test
  
  // Center servos
  servoX.write(90);
  servoY.write(90);
  delay(500);
  
  // Prepare for test
  Serial.println(F("Testing X-axis response time..."));
  
  // Record start time
  long startTime = micros();
  
  // Move servo quickly to test angle
  servoX.write(90 + RESPONSE_TEST_ANGLE);
  
  // Monitor rotation until target angle is reached
  float targetAngle = RESPONSE_TEST_ANGLE;
  float currentAngle = 0;
  float peakAngle = 0;
  bool targetReached = false;
  long responseTime = 0;
  
  // Check response for up to 500ms
  for (int i = 0; i < 500; i++) {
    // Read sensor
    Vector normGyro = mpu.readNormalizeGyro();
    Vector normAccel = mpu.readNormalizeAccel();
    
    // Simple complementary filter for angle estimation
    float accelAngleX = atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0 / PI;
    currentAngle = 0.98 * (currentAngle + normGyro.XAxis * 0.001) + 0.02 * accelAngleX;
    
    // Update peak
    if (abs(currentAngle) > abs(peakAngle)) {
      peakAngle = currentAngle;
    }
    
    // Check if we've reached target
    if (!targetReached && abs(currentAngle) >= targetAngle * 0.9) {
      responseTime = (micros() - startTime) / 1000.0; // convert to milliseconds
      targetReached = true;
      break;
    }
    
    delay(1);
  }
  
  // Return servo to center
  servoX.write(90);
  
  // Report results
  Serial.print(F("Response time: "));
  if (targetReached) {
    Serial.print(responseTime);
    Serial.println(F(" ms"));
    
    // Compare to expected value
    float percentDiff = (responseTime - TARGET_RESPONSE_TIME) / TARGET_RESPONSE_TIME * 100.0;
    Serial.print(F("Difference from target ("));
    Serial.print(TARGET_RESPONSE_TIME);
    Serial.print(F("ms): "));
    Serial.print(percentDiff);
    Serial.println(F("%"));
    
    if (abs(percentDiff) < 25.0) {
      Serial.println(F("RESULT: Response time within acceptable range."));
    } else {
      Serial.println(F("RESULT: Response time significantly different from expected. Check hardware."));
    }
  } else {
    Serial.println(F("Did not reach target angle within timeout period."));
    Serial.println(F("RESULT: Response test failed. Check servo operation and power."));
  }
  
  // Additional info
  Serial.print(F("Peak angle reached: "));
  Serial.print(peakAngle);
  Serial.println(F(" degrees"));
}

void testStability() {
  Serial.println(F("\n--- Test 3: Stability Test ---"));
  setLED(0, 255, 255); // Cyan during stability test
  
  Serial.println(F("Testing orientation stability... Please keep the system still."));
  
  // Center servos
  servoX.write(90);
  servoY.write(90);
  delay(500);
  
  // Variables for tracking stability
  float startAngles[3] = {0, 0, 0};
  float maxDeviation[3] = {0, 0, 0};
  float avgDeviation[3] = {0, 0, 0};
  
  // Get initial orientation
  updateOrientation();
  for (int i = 0; i < 3; i++) {
    startAngles[i] = currentAngles[i];
  }
  
  // Track stability over time
  for (int sample = 0; sample < NUM_SAMPLES; sample++) {
    updateOrientation();
    
    // Calculate deviation from starting position
    for (int i = 0; i < 3; i++) {
      float deviation = abs(currentAngles[i] - startAngles[i]);
      avgDeviation[i] += deviation;
      if (deviation > maxDeviation[i]) {
        maxDeviation[i] = deviation;
      }
    }
    
    delay(100);
  }
  
  // Calculate averages
  for (int i = 0; i < 3; i++) {
    avgDeviation[i] /= NUM_SAMPLES;
  }
  
  // Report results
  Serial.println(F("Stability test results:"));
  Serial.println(F("Axis    | Avg Deviation | Max Deviation"));
  Serial.println(F("--------|---------------|---------------"));
  
  Serial.print(F("Pitch   | "));
  Serial.print(avgDeviation[0], 3);
  Serial.print(F("° | "));
  Serial.print(maxDeviation[0], 3);
  Serial.println(F("°"));
  
  Serial.print(F("Roll    | "));
  Serial.print(avgDeviation[1], 3);
  Serial.print(F("° | "));
  Serial.print(maxDeviation[1], 3);
  Serial.println(F("°"));
  
  Serial.print(F("Yaw     | "));
  Serial.print(avgDeviation[2], 3);
  Serial.print(F("° | "));
  Serial.print(maxDeviation[2], 3);
  Serial.println(F("°"));
  
  // Evaluate results
  bool stabilityOK = true;
  for (int i = 0; i < 3; i++) {
    if (avgDeviation[i] > 1.5 || maxDeviation[i] > 3.0) {
      stabilityOK = false;
    }
  }
  
  if (stabilityOK) {
    Serial.println(F("RESULT: System demonstrates good stability."));
  } else {
    Serial.println(F("RESULT: Stability issues detected. Check sensor mounting and environmental factors."));
  }
}

void reportOverallStatus() {
  Serial.println(F("\n=== Overall System Status ==="));
  
  Serial.println(F("System is operational."));
  Serial.println(F("Recommended next steps:"));
  Serial.println(F("1. Run PID tuning if response time needs adjustment"));
  Serial.println(F("2. Monitor battery levels during operation"));
  Serial.println(F("3. Check servo temperature after extended use"));
  
  // Set LED to green to indicate completion
  setLED(0, 255, 0);
}

// Helper Functions

void updateOrientation() {
  // Read raw sensor data
  Vector normGyro = mpu.readNormalizeGyro();
  Vector normAccel = mpu.readNormalizeAccel();
  
  // Calculate angles from accelerometer
  float accelAngleX = atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0 / PI;
  float accelAngleY = atan2(normAccel.XAxis, normAccel.ZAxis) * 180.0 / PI;
  
  // Time tracking for integration
  static unsigned long lastUpdateTime = 0;
  unsigned long now = millis();
  float dt = (now - lastUpdateTime) / 1000.0;
  lastUpdateTime = now;
  if (dt > 0.1) dt = 0.1; // Cap at 100ms to prevent large jumps
  
  // Simple complementary filter
  currentAngles[0] = 0.98 * (currentAngles[0] + normGyro.XAxis * dt) + 0.02 * accelAngleX;
  currentAngles[1] = 0.98 * (currentAngles[1] + normGyro.YAxis * dt) + 0.02 * accelAngleY;
  currentAngles[2] += normGyro.ZAxis * dt; // Yaw (no accelerometer correction)
  
  // Simple Kalman filter implementation
  for (int i = 0; i < 3; i++) {
    // Prediction
    float prediction = previousEstimate[i];
    estimateUncertainty += processNoise;
    
    // Update
    float kalmanGain = estimateUncertainty / (estimateUncertainty + measurementNoise);
    kalmanAngles[i] = prediction + kalmanGain * (currentAngles[i] - prediction);
    estimateUncertainty = (1 - kalmanGain) * estimateUncertainty;
    
    // Save for next iteration
    previousEstimate[i] = kalmanAngles[i];
  }
}

void setLED(int r, int g, int b) {
  analogWrite(LED_R_PIN, r);
  analogWrite(LED_G_PIN, g);
  analogWrite(LED_B_PIN, b);
}
