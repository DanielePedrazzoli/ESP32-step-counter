#ifndef _BLE_MANAGER_
#define _BLE_MANAGER_

#include <NimBLEDevice.h>
#include "SensorData.h"

union float2bytes
{
    float f;
    uint8_t b[sizeof(float)];
};

class BLE_Manager
{
public:
    BLE_Manager();

    int init(void);

    void changeStepValue(int);
    void changeMotionState(int);
    void changeValue_Accelerometer_value(SensorData *);

private:
    /* UUDI dei servizi */
    static const char *const SENSOR_UUID;
    static const char *const DATA_UUID;

    /* UUDI delle caratteristiche raw */
    static const char *const ACCELEROMETER_UUID;

    /* UUDI delle caratteristiche analizzate */
    static const char *const STEP_UUID;
    static const char *const STATE_UUID;

    static const char *const DEVICE_NAME;

    NimBLEServer *pServer;

    NimBLEService *pSensorService;
    NimBLECharacteristic *pAccelerometerData;

    NimBLEService *pDataService;
    NimBLECharacteristic *pStepCout;
    NimBLECharacteristic *pCurrentState;
};

#endif