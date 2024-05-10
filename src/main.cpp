
#include <Arduino.h>
#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps612.h>
#include "Wire.h"
#include "BLEManager.h"
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

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorFloat gravity; // [x, y, z]            gravity vector
Quaternion q;        // [w, x, y, z]         quaternion container

volatile bool mpuInterrupt = false;

/**
 * @brief Funzione di interrupt richiamata ogni volta che MPU6050 effettua un campionamento
 *
 */
void dmpDataReady()
{
  mpuInterrupt = true;
}

/**
 * @brief Funzione di inizializzazione di tutte le seriali usate
 *
 */
void initSerial()
{
  Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_FREQUENCY);
  Serial.begin(115200);
  while (!Serial)
    ;
}

/**
 * @brief Inizializzaione sensore e DMP del sensore
 *
 * @return int codice di errore.
 * - 0 nesusn errore
 * - 1 initial memory load failed
 * - 2 DMP configuration updates failed
 */
int initSensor()
{
  // initialize device
  Serial.println(F("Inizializazione sensore in corso..."));
  mpu.initialize();

  // Verifica della connessione
  Serial.println(F("Connessione a MPU6050 "));
  Serial.print(mpu.testConnection() ? F("avvenuta") : F("fallita"));

  // load and configure the DMP
  Serial.println(F("Inizializzazione DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setIntDataReadyEnabled(true);
  Serial.println(F("Abilitazione interrupt"));
  attachInterrupt(INTERRUPT_Sensor, dmpDataReady, RISING);

  if (devStatus == 0)
  {
    Serial.println(F("Abilitazione DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP pronto"));
    packetSize = mpu.dmpGetFIFOPacketSize();

    return 0;
  }
  else
  {
    Serial.print(F("Inizializzaione DMP fallita (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    return devStatus;
  }
}

/**
 * @brief Main setup
 *
 */
void setup()
{
  // Abilitazione del pin dell'autocalibrazione
  pinMode(25, INPUT_PULLDOWN);

  initSerial();
  dmpReady = !initSensor();

  accelerometerData.init();
  calManager.importCalibrationData();

  I2Cdev::writeByte(0x68, 0x19, 39);         // 200, Sample frequency = 8000 / (1 + n)
  I2Cdev::writeByte(0x68, 0x1A, 0b00000000); // DLPF
  mpu.resetFIFO();

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

/**
 * @brief main loop
 *
 */
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
    Serial.print("Passo rilevato. Totale: ");
    Serial.println(motionSensor.step);
  }

  bleManager.changeValue_Accelerometer_value(&accelerometerData);
}
