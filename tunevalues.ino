#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

int const INTERRUPT_PIN = 2;
bool DMPReady = false;
uint8_t devStatus;
uint8_t FIFOBuffer[64];

VectorInt16 gy;

volatile bool MPUInterrupt = false;
void DMPDataReady() {
  MPUInterrupt = true;
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

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  if(mpu.testConnection() == false) {
    while(true);
  }

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);

    DMPReady = true;
  } else {
    while(true);
  }
}

void loop() {
  if (!DMPReady) return;
  
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetGyro(&gy, FIFOBuffer);

    Serial.print("Gyro X: ");
    Serial.print(gy.x);
    Serial.print("\tGyro Y: ");
    Serial.print(gy.y);
    Serial.print("\tGyro Z: ");
    Serial.println(gy.z);
  }
}
