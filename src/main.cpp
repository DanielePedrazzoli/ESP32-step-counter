
#include <Arduino.h>
#include "BLEManager.h"
#include "MPU.h"
#include "I2Cdev.h"

#include "Costants.h"

BLE_Manager bleManager;
SensorData accelerometerData;
SensorData gyroscopeData;

volatile bool dateReady = false;

void IRAM_ATTR interruptSensor()
{
  dateReady = true;
}

#define I2C_MASTER_SCL_IO 19
#define I2C_MASTER_SDA_IO 18
#define VCC_SENSOR 21
#define XDA_SENSOR 5
#define XCL_SENSOR 17
#define AD0_SENSOR 16
#define INTERRUPT_Sensor 4

void setup()
{

  // Pin mode per garantire l'attacco della scheda del sensore all'ESP32 diretto senza fili
  pinMode(VCC_SENSOR, OUTPUT);
  pinMode(XDA_SENSOR, INPUT); // tristate
  pinMode(XCL_SENSOR, INPUT); // tristate
  pinMode(AD0_SENSOR, INPUT); // tristate
  digitalWrite(VCC_SENSOR, HIGH);

  accelerometerData.init();
  // gyroscopeData.initFilter();

  Serial.begin(115200);
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
  conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

  initMPU();
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_Sensor), interruptSensor, RISING);

  bleManager.init();
}

void loop()
{

  // 3 valori presenti dentro il buffer FIFO sono composti da 6 byte totali
  // 2 byte per ogni asse
  // Devo quindi impostare il treshoold con un numero divisibile per 6, così
  // evitare di avere dati incompleti
  // Allo stesso tempo il buffer FIFO ha una capacità massima di 1024 byte e non avrebbe
  // senso rischiare di andare in overflow,poiché questo sfaserebbe i risultati
  // Quindi i limiti di massima lettura sono dati da:
  // - divisibilità per 6
  // - minore o uguale di 900 (così da lasciare un po' di margine al buffer)
  // Se ad esempio prendessimo 600, ogni lettura otterremmo 10 valori per l'asse x
  // 10 valori per l'asse y e 10 valori per l'asse z

  if (dateReady)
  {

    dateReady = false;

    // data 6 & 7 are temp, must ignore them
    uint8_t buffer[6];

    I2Cdev::readBytes(SENSOR_ADDRESS, 0x3B, 14, buffer);

    accelerometerData.computeMagnitude(((int16_t)buffer[0] << 8) | buffer[1], ((int16_t)buffer[2] << 8) | buffer[3], ((int16_t)buffer[4] << 8) | buffer[5]);

    // gyroscopeData.addValue(AXES::X, ((int16_t)buffer[8] << 8) | buffer[9]);
    // gyroscopeData.addValue(AXES::Y, ((int16_t)buffer[10] << 8) | buffer[11]);
    // gyroscopeData.addValue(AXES::Z, ((int16_t)buffer[12] << 8) | buffer[13]);

    bleManager.changeValue_Accelerometer_value(&accelerometerData);
  }

  delay(100);
}
