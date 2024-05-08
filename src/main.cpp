
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

Preferences preferences;
MPU6050 mpu(0x68);
CalibrationManager calManager(&mpu, &preferences);
BLE_Manager bleManager;
SensorData accelerometerData;

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
uint16_t fifoCount;     // count of all bytes currently in FIFO
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
    return;
  }

  calManager.importCalibrationData();

  // I2Cdev::writeByte(0x68, 0x6B, 0b00000001); // Internal Clock set to Gyro output
  // I2Cdev::writeByte(0x68, 0x6A, 0b00000100); // Reset FIFO
  // I2Cdev::writeByte(0x68, 0x19, 79);         // 100, Sample frequency = 8000 / (1 + n)
  // I2Cdev::writeByte(0x68, 0x1A, 0b00000000); // DLPF
  // I2Cdev::writeByte(0x68, 0x23, 0b01111000); // Load Accelerometer (3° bit) & gyro (6° = x, 5° = y, 4° = z bit) to FIFO
  // I2Cdev::writeByte(0x68, 0x38, 0b00000001); // Data ready Interrupt
  // I2Cdev::writeByte(0x68, 0x6A, 0b01000100); // Enable FIFO
  // mpu.setFullScaleAccelRange(0x00); // Accelerometer sensitivity (1g)
  mpu.resetFIFO();
  // mpu.setDLPFMode();

  bleManager.init();
}

void loop()
{

  if (!dmpReady)
    return;

  if (!mpuInterrupt)
    return;

  mpuInterrupt = false;

  while (!mpuInterrupt && fifoCount < packetSize)
  {
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }

  if (!(mpuIntStatus & 0x02))
  {
    return;
  }

  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount();

  //     // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  //     // track FIFO count here in case there is > 1 packet available
  //     // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  // #ifdef OUTPUT_READABLE_YAWPITCHROLL
  //     mpu.dmpGetQuaternion(&q, fifoBuffer);
  //     mpu.dmpGetGravity(&gravity, &q);
  //     // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  //     // Serial.print("ypr\t");
  //     // Serial.print(ypr[0] * 180 / M_PI);
  //     // Serial.print("\t");
  //     // Serial.print(ypr[1] * 180 / M_PI);
  //     // Serial.print("\t");
  //     // Serial.print(ypr[2] * 180 / M_PI);
  //     Serial.print("\tpacketSize\t");
  //     Serial.print(packetSize);
  //     Serial.print("\tGravity\t");
  //     Serial.print(gravity.x);
  //     Serial.print("\t");
  //     Serial.print(gravity.y);
  //     Serial.print("\t");
  //     Serial.println(gravity.z);
  // #endif

  // Serial.println(F("loading data"));
  VectorInt16 accVector;
  VectorInt16 accRawVector;

  // Serial.println(F("get quaterions"));
  // mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  // Serial.println(F("get gravity"));
  mpu.dmpGetGravity(&gravity, &q);
  // Serial.println(F("get accell"));
  mpu.dmpGetAccel(&accRawVector, fifoBuffer);

  // Serial.println(F("get linear accell"));
  mpu.dmpGetLinearAccel(&accVector, &accRawVector, &gravity);
  // Serial.print(gravity.x);
  // Serial.print("\t");
  // Serial.print(gravity.y);
  // Serial.print("\t");
  // Serial.println(gravity.z);

  // Serial.println(F("save data"));
  accelerometerData.computeMagnitude(&accVector);

  // Serial.println(F("send data"));
  bleManager.changeValue_Accelerometer_value(&accelerometerData);
  // }

  //   // wait for MPU interrupt or extra packet(s) available
  //   while (!mpuInterrupt && fifoCount < packetSize)
  //   {
  //   }

  //   // reset interrupt flag and get INT_STATUS byte
  //   mpuInterrupt = false;
  //   mpuIntStatus = mpu.getIntStatus();

  //   // get current FIFO count
  //   fifoCount = mpu.getFIFOCount();

  //   // check for overflow (this should never happen unless our code is too inefficient)
  //   if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  //   {
  //     // reset so we can continue cleanly
  //     mpu.resetFIFO();
  //     Serial.println(F("FIFO overflow!"));
  //   }
  //   else if (mpuIntStatus & 0x02)
  //   {
  //     // wait for correct available data length, should be a VERY short wait
  //     while (fifoCount < packetSize)
  //       fifoCount = mpu.getFIFOCount();

  //     // read a packet from FIFO
  //     mpu.getFIFOBytes(fifoBuffer, packetSize);

  //     // track FIFO count here in case there is > 1 packet available
  //     // (this lets us immediately read more without waiting for an interrupt)
  //     fifoCount -= packetSize;

  // #ifdef OUTPUT_READABLE_YAWPITCHROLL
  //     mpu.dmpGetQuaternion(&q, fifoBuffer);
  //     mpu.dmpGetGravity(&gravity, &q);
  //     // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  //     // Serial.print("ypr\t");
  //     // Serial.print(ypr[0] * 180 / M_PI);
  //     // Serial.print("\t");
  //     // Serial.print(ypr[1] * 180 / M_PI);
  //     // Serial.print("\t");
  //     // Serial.print(ypr[2] * 180 / M_PI);
  //     Serial.print("\tpacketSize\t");
  //     Serial.print(packetSize);
  //     Serial.print("\tGravity\t");
  //     Serial.print(gravity.x);
  //     Serial.print("\t");
  //     Serial.print(gravity.y);
  //     Serial.print("\t");
  //     Serial.println(gravity.z);
  // #endif
  //   }
}
