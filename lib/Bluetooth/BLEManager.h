#ifndef _BLE_MANAGER_
#define _BLE_MANAGER_

#include "NimBLEDevice.h"
#include "SensorData.h"
#include "debug.h"

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
    void updateAnalisysData(float, float);

#if DEBUG_MODE
    void changeValue_Accelerometer_value(SensorData *);
#endif

private:
    /* UUDI dei servizi */
    static const char *const DATA_UUID;
#if DEBUG_MODE
    static const char *const SENSOR_UUID;
#endif

/* UUDI delle caratteristiche raw */
#if DEBUG_MODE
    static const char *const ACCELEROMETER_UUID;
#endif

    /* UUDI delle caratteristiche analizzate */
    static const char *const STEP_UUID;
    static const char *const FREQUENCY_UUID;
    static const char *const DISTANCE_UUID;

    static const char *const DEVICE_NAME;

    NimBLEServer *pServer;

#if DEBUG_MODE
    NimBLEService *pSensorService;
    NimBLECharacteristic *pAccelerometerData;
#endif

    NimBLEService *pDataService;
    NimBLECharacteristic *pStepCout;
    NimBLECharacteristic *pStepFrequency;
    NimBLECharacteristic *pDistanceTravelled;
};

#endif