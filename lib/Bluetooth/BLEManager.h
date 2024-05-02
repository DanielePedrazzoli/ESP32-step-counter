#pragma once

#include "SensorData.h"
#include <NimBLEDevice.h>

class BLE_Manager
{
public:
    BLE_Manager();
    ~BLE_Manager();

    int init(void);

    void changeValue_step(int);
    void changeValue_state(int);
    void changeValue_Accelerometer_value(SensorData *);
    void changeValue_Gyroscope_value(SensorData *);
    void split16to8(int16_t *, uint8_t *, uint8_t);

private:
    /* UUDI dei servizi */
    static const char *const SENSOR_UUID;
    static const char *const DATA_UUID;

    /* UUDI delle caratteristiche raw */
    static const char *const ACCELEROMETER_UUID;
    static const char *const GYROSCOPE_UUID;

    /* UUDI delle caratteristiche analizzate */
    static const char *const STEP_UUID;
    static const char *const STATE_UUID;

    static const char *const DEVICE_NAME;

    NimBLEServer *pServer;

    NimBLEService *pSensorService;
    NimBLECharacteristic *pAccelerometerData;
    // NimBLECharacteristic *pAccelerometerData_Y;
    // NimBLECharacteristic *pAccelerometerData_Z;
    NimBLECharacteristic *pGyroscopeData;

    NimBLEService *pDataService;
    NimBLECharacteristic *pStepCout;
    NimBLECharacteristic *pCurrentState;
};