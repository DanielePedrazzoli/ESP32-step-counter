
#include "debug.h"
#include <Arduino.h>
#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps612.h>
#include "Wire.h"
#include "BLEManager.h"
#include <driver/i2c.h>
#include "Calibration/CalibrationManager.h"
#include "MotionSensor.h"

uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t *packet);
uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
uint8_t dmpGetAccel(VectorInt16 *v, const uint8_t *packet);
uint8_t dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);

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

volatile uint8_t packetCount = 0;
uint8_t precPacketCount = 0;
uint8_t packetCountLimit = 0;

hw_timer_t *StepAnalisyTimer = NULL;
bool analizeSteps = false;

void IRAM_ATTR onTimer()
{
  analizeSteps = true;
}

/**
 * @brief Funzione di interrupt richiamata ogni volta che MPU6050 effettua un campionamento
 *
 */
void IRAM_ATTR dmpDataReady()
{

  packetCount++;
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
    Serial.print(F("packetSize: "));
    Serial.println(packetSize);

    // determino il numero massimo di valori che possono essere salvati nella FIFO
    // in base alla loro dimensione
    // La dimensione della FIFO è 1024 byte e per sicurezza è meglio avere dei
    // pacchetti di scarto, quindi la quantità massima è
    int padding = 2;
    packetCountLimit = 1; //(int)(1024 / packetSize - padding);
#ifdef DEBUG
    Serial.print(F("count limit: "));
    Serial.println(packetCountLimit);
#endif

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
  pinMode(15, INPUT_PULLUP);

  initSerial();
  dmpReady = !initSensor();

  accelerometerData.init();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  calManager.importCalibrationData();

  I2Cdev::writeByte(0x68, 0x19, 39);         // 200, Sample frequency = 8000 / (1 + n) 39
  I2Cdev::writeByte(0x68, 0x1A, 0b00000000); // DLPF disabilitato
  mpu.resetFIFO();

  bleManager.init();
  motionSensor.init(&bleManager);

  // timer 0 con un divisore di 80 e conteggio crescente (0,1,2,3,4,...)
  StepAnalisyTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(StepAnalisyTimer, &onTimer, true);

  // Clock ESP32 = 80MHz --> divder a 80 --> 1Mhz = 1μs ---> 2000000 = 2s
  timerAlarmWrite(StepAnalisyTimer, 2000000, true);
  timerAlarmEnable(StepAnalisyTimer);
}

/**
 * @brief Lettura dei valori dal sensore
 */
void readData()
{

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

  int fifoCount = 0;
  while (fifoCount < packetSize * packetCountLimit)
    fifoCount = mpu.getFIFOCount();

  // Serial.print("Reading data\nTotal FIFO count: ");
  // Serial.println(fifoCount);

  // Per ogni pacchetto avvito la seqeunza di parsing e salvo i valori
  for (int i = 0; i < packetCountLimit; i++)
  {
    // Essendo già il  buffer una FIFO non è necessario tenere conto di quale pacchetto
    // sto andando a leggere. Il pacchetto sarà sempre disponibile come primo
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    VectorInt16 accVector;
    VectorInt16 accRawVector;
    VectorFloat gravity;
    Quaternion q;

    dmpGetQuaternion(&q, fifoBuffer);
    dmpGetGravity(&gravity, &q);
    dmpGetAccel(&accRawVector, fifoBuffer);
    dmpGetLinearAccel(&accVector, &accRawVector, &gravity);

    bool stepAdded = accelerometerData.pushValue(&accVector);
    if (stepAdded)
    {
      motionSensor.addStep(&accelerometerData.mosteRecentStep);
      Serial.print("Passo rilevato. Totale: ");
      Serial.println(motionSensor.step);
    }

#if DEBUG_MODE
    bleManager.changeValue_Accelerometer_value(&accelerometerData);
#endif
  }
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

  if (analizeSteps)
  {
    analizeSteps = false;
    motionSensor.sampleStep(&accelerometerData);
  }

  // if (precPacketCount != packetCount)
  // {
  //   Serial.println(packetCount);
  //   precPacketCount = packetCount;
  // }

  if (packetCount >= packetCountLimit)
  {
    readData();
    packetCount = 0;
  }
}

/**************************************************************************************************************************/
/**************************************************************************************************************************/
/**************************************************************************************************************************/

/**
 * @brief Estrae i quaternione dal pacchetto inviato dal DMP e lo compone
 *
 * @param q
 * @param packet
 * @return uint8_t
 */
uint8_t dmpGetQuaternion(Quaternion *quaternion, const uint8_t *packet)
{
  int16_t qI[4];
  qI[0] = ((packet[0] << 8) | packet[1]);
  qI[1] = ((packet[4] << 8) | packet[5]);
  qI[2] = ((packet[8] << 8) | packet[9]);
  qI[3] = ((packet[12] << 8) | packet[13]);

  quaternion->w = (float)qI[0] / (16384.0f);
  quaternion->x = (float)qI[1] / (16384.0f);
  quaternion->y = (float)qI[2] / (16384.0f);
  quaternion->z = (float)qI[3] / (16384.0f);

  return 0;
}

/**
 * @brief Applica la moltiplicazione di quaternioni per eliminare la componente gravitazionale dalle accellerazioni ottenute
 *
 * @param v
 * @param q
 * @return uint8_t
 */
uint8_t dmpGetGravity(VectorFloat *resultVector, Quaternion *quaternion)
{
  float qx = quaternion->x;
  float qy = quaternion->y;
  float qz = quaternion->z;
  float qw = quaternion->w;

  resultVector->x = 2 * (qx * qz - qw * qy);
  resultVector->y = 2 * (qw * qx + qy * qz);
  resultVector->z = qw * qw - qx * qx - qy * qy + qz * qz;
  return 0;
}

/**
 * @brief Estrae i valori di accelerazione dal pacchetto inviato dal DMP
 *
 * @param v
 * @param packet
 * @return uint8_t
 */
uint8_t dmpGetAccel(VectorInt16 *resultVector, const uint8_t *packet)
{
  resultVector->x = (packet[16] << 8) | packet[17];
  resultVector->y = (packet[18] << 8) | packet[19];
  resultVector->z = (packet[20] << 8) | packet[21];
  return 0;
}

/**
 * @brief Elimina la componente gravitazionale [gravity] dal vettore di partenza [vRaw]
 *
 * @param v
 * @param vRaw
 * @param gravity
 * @return uint8_t
 */
uint8_t dmpGetLinearAccel(VectorInt16 *linearAcceleration, VectorInt16 *rawAcceleration, VectorFloat *gravity)
{
  // get rid of the gravity component (+1g = +16384 in standard DMP FIFO packet, sensitivity is 2g)
  linearAcceleration->x = rawAcceleration->x - gravity->x * (16384);
  linearAcceleration->y = rawAcceleration->y - gravity->y * (16384);
  linearAcceleration->z = rawAcceleration->z - gravity->z * (16384);
  return 0;
}
