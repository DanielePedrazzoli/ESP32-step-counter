
#include <Arduino.h>
#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps612.h>
#include "Wire.h"

#include "BLEManager.h"

#include "Costants.h"
#include <driver/i2c.h>
#include "Calibration/CalibrationManager.h"
#include "MotionSensor.h"

Preferences preferences;
MPU6050 mpu(0x68);
CalibrationManager calManager(&mpu, &preferences);
BLE_Manager bleManager;
SensorData accelerometerData;
MotionSensor motionSensor;

#define I2C_MASTER_SCL_IO 19
#define I2C_MASTER_SDA_IO 18
#define XDA_SENSOR 5
#define XCL_SENSOR 17
#define AD0_SENSOR 16
#define INTERRUPT_Sensor 4
#define OUTPUT_READABLE_YAWPITCHROLL
#define I2C_FREQUENCY 400000

#define CALIBRATION_ON false

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorFloat gravity; // [x, y, z]            gravity vector
Quaternion q;        // [w, x, y, z]         quaternion container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  pinMode(25, INPUT_PULLDOWN);
  Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_FREQUENCY);

  Serial.begin(115200);
  while (!Serial)
    ;

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(F("MPU6050 connection "));
  Serial.print(mpu.testConnection() ? F("successful") : F("failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setIntDataReadyEnabled(true);
  Serial.println(F("Enabling interrupt detection"));
  attachInterrupt(INTERRUPT_Sensor, dmpDataReady, RISING);
  accelerometerData.init();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready"));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  calManager.importCalibrationData();

  // I2Cdev::writeByte(0x68, 0x6B, 0b00000001); // Internal Clock set to Gyro output
  // I2Cdev::writeByte(0x68, 0x6A, 0b00000100); // Reset FIFO
  I2Cdev::writeByte(0x68, 0x19, 39);         // 200, Sample frequency = 8000 / (1 + n)
  I2Cdev::writeByte(0x68, 0x1A, 0b00000000); // DLPF
  // I2Cdev::writeByte(0x68, 0x23, 0b01111000); // Load Accelerometer (3° bit) & gyro (6° = x, 5° = y, 4° = z bit) to FIFO
  // I2Cdev::writeByte(0x68, 0x38, 0b00000001); // Data ready Interrupt
  // I2Cdev::writeByte(0x68, 0x6A, 0b01000100); // Enable FIFO
  // mpu.setFullScaleAccelRange(0x00); // Accelerometer sensitivity (1g)
  mpu.resetFIFO();
  // mpu.setDLPFMode();

  bleManager.init();
  motionSensor.init(&bleManager);
}

void printVector(VectorFloat v)
{
  Serial.print(v.x);
  Serial.print("\t");
  Serial.print(v.y);
  Serial.print("\t");
  Serial.print(v.z);
  Serial.println("");
}

void printVector(VectorInt16 v)
{
  Serial.print(v.x);
  Serial.print("\t");
  Serial.print(v.y);
  Serial.print("\t");
  Serial.print(v.z);
  Serial.println("");
}

void loop()
{

  if (!dmpReady)
  {
    return;
  }

  if (!mpuInterrupt)
  {
    return;
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  if ((mpuIntStatus & 0x10))
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (!(mpuIntStatus & 0x02))
  {
    return;
  }

  // wait for correct available data length, should be a VERY short wait
  int fifoCount = 0;
  while (fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount();

  mpu.getFIFOBytes(fifoBuffer, packetSize);

  VectorInt16 accVector;
  VectorInt16 accRawVector;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetAccel(&accRawVector, fifoBuffer);

  mpu.dmpGetLinearAccel(&accVector, &accRawVector, &gravity);

  bool stepAdded = accelerometerData.addValue(&accVector);
  if (stepAdded)
  {
    motionSensor.addStep();
    Serial.print("step added. Total: ");
    Serial.println(motionSensor.step);
  }

  bleManager.changeValue_Accelerometer_value(&accelerometerData);
}
