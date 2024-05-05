
#include <Arduino.h>
#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps612.h>
#include "Wire.h"

#include "BLEManager.h"
#include "Costants.h"
#include <driver/i2c.h>
#include "Calibration/CalibrationManager.h"

// BLE_Manager bleManager;
// SensorData accelerometerData;
// SensorData gyroscopeData;

// volatile bool dateReady = false;

// void IRAM_ATTR interruptSensor()
// {
//   dateReady = true;
// }

// #define I2C_MASTER_SCL_IO 19
// #define I2C_MASTER_SDA_IO 18
// #define VCC_SENSOR 21
// #define XDA_SENSOR 5
// #define XCL_SENSOR 17
// #define AD0_SENSOR 16
// #define INTERRUPT_Sensor 4

// void setup()
// {

//   // Pin mode per garantire l'attacco della scheda del sensore all'ESP32 diretto senza fili
//   pinMode(VCC_SENSOR, OUTPUT);
//   pinMode(XDA_SENSOR, INPUT); // tristate
//   pinMode(XCL_SENSOR, INPUT); // tristate
//   pinMode(AD0_SENSOR, INPUT); // tristate
//   digitalWrite(VCC_SENSOR, HIGH);

//   accelerometerData.init();
//   // gyroscopeData.initFilter();

//   Serial.begin(115200);
//   i2c_config_t conf;
//   conf.mode = I2C_MODE_MASTER;
//   conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
//   conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
//   conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//   conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//   conf.master.clk_speed = 400000;
//   ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
//   ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

//   initMPU();
//   attachInterrupt(digitalPinToInterrupt(INTERRUPT_Sensor), interruptSensor, RISING);

//   bleManager.init();
// }

// void loop()
// {

//   // 3 valori presenti dentro il buffer FIFO sono composti da 6 byte totali
//   // 2 byte per ogni asse
//   // Devo quindi impostare il treshoold con un numero divisibile per 6, così
//   // evitare di avere dati incompleti
//   // Allo stesso tempo il buffer FIFO ha una capacità massima di 1024 byte e non avrebbe
//   // senso rischiare di andare in overflow,poiché questo sfaserebbe i risultati
//   // Quindi i limiti di massima lettura sono dati da:
//   // - divisibilità per 6
//   // - minore o uguale di 900 (così da lasciare un po' di margine al buffer)
//   // Se ad esempio prendessimo 600, ogni lettura otterremmo 10 valori per l'asse x
//   // 10 valori per l'asse y e 10 valori per l'asse z

//   if (dateReady)
//   {

//     dateReady = false;

//     // data 6 & 7 are temp, must ignore them
//     uint8_t buffer[6];

//     I2Cdev::readBytes(SENSOR_ADDRESS, 0x3B, 14, buffer);

//     accelerometerData.computeMagnitude(((int16_t)buffer[0] << 8) | buffer[1], ((int16_t)buffer[2] << 8) | buffer[3], ((int16_t)buffer[4] << 8) | buffer[5]);

//     // gyroscopeData.addValue(AXES::X, ((int16_t)buffer[8] << 8) | buffer[9]);
//     // gyroscopeData.addValue(AXES::Y, ((int16_t)buffer[10] << 8) | buffer[11]);
//     // gyroscopeData.addValue(AXES::Z, ((int16_t)buffer[12] << 8) | buffer[13]);

//     bleManager.changeValue_Accelerometer_value(&accelerometerData);
//   }

//   delay(100);
// }
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

Preferences preferences;
MPU6050 mpu(0x68);

#define I2C_MASTER_SCL_IO 19
#define I2C_MASTER_SDA_IO 18
#define XDA_SENSOR 5
#define XCL_SENSOR 17
#define AD0_SENSOR 16
#define INTERRUPT_Sensor 4
#define OUTPUT_READABLE_YAWPITCHROLL

#define CALIBRATION_ON false

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorFloat gravity; // [x, y, z]            gravity vector
Quaternion q;        // [w, x, y, z]         quaternion container
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

CalibrationManager calManager(&mpu, &preferences);

void setup()
{
  pinMode(XDA_SENSOR, INPUT); // tristate
  pinMode(XCL_SENSOR, INPUT); // tristate
  pinMode(AD0_SENSOR, INPUT); // tristate
  // i2c_config_t conf;
  // conf.mode = I2C_MODE_MASTER;
  // conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
  // conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
  // conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  // conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  // conf.master.clk_speed = 400000;
  // ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  // ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
  Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 400000);

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

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ; // empty buffer
  while (!Serial.available())
    ; // wait for data
  while (Serial.available() && Serial.read())
    ; // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // mpu.setXGyroOffset(220);
  // mpu.setYGyroOffset(76);
  // mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(INTERRUPT_Sensor, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

  // configure LED for output
  // Usando i dati ottenuti dalla calibrazione
  // accelgyro.setXAccelOffset(63);
  // accelgyro.setYAccelOffset(-1302);
  // accelgyro.setZAccelOffset(1082);
  // accelgyro.setXGyroOffset(238);
  // accelgyro.setYGyroOffset(-43);
  // accelgyro.setZGyroOffset(28);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{

  if (CALIBRATION_ON)
  {
    Serial.println("Sensor calibration");
    calManager.startCalibration();
    while (1)
      ;
  }

  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\tGravity\t");
    Serial.print(gravity.x);
    Serial.print("\t");
    Serial.print(gravity.y);
    Serial.print("\t");
    Serial.println(gravity.z);
#endif
  }
}
