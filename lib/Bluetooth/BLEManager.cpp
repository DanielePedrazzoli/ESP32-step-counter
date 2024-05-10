#include "BLEManager.h"

const char *const BLE_Manager::DEVICE_NAME = "ESP 32 motion sensor";

const char *const BLE_Manager::SENSOR_UUID = "130db0c9-e717-45ba-8a99-21d03144f056";
const char *const BLE_Manager::DATA_UUID = "2f766cb0-804b-4779-ba00-80a3f08cc70e";
const char *const BLE_Manager::ACCELEROMETER_UUID = "7c15c5fb-f9a8-41d2-885f-07db1827370f";
const char *const BLE_Manager::STEP_UUID = "3595f562-1fa2-45d1-9f77-81d9e70aebca";
const char *const BLE_Manager::STATE_UUID = "c8754d82-859f-4724-b6e5-a2e7cd7f7479";

BLE_Manager::BLE_Manager() {}

int BLE_Manager::init()
{
    NimBLEDevice::init(DEVICE_NAME);
    pServer = NimBLEDevice::createServer();

    /* Inizializzazione service sensori */
    pSensorService = pServer->createService(BLEUUID(SENSOR_UUID));
    pAccelerometerData = pSensorService->createCharacteristic(ACCELEROMETER_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pSensorService->start();

    /* Inizializzazione service dati */
    pDataService = pServer->createService(BLEUUID(DATA_UUID));
    pStepCout = pDataService->createCharacteristic(STEP_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pCurrentState = pDataService->createCharacteristic(STATE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pDataService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SENSOR_UUID);
    pAdvertising->addServiceUUID(DATA_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();
    return 1;
}

void BLE_Manager::changeStepValue(int newValue)
{
    pStepCout->setValue(newValue);
    pStepCout->notify();
}

void BLE_Manager::changeMotionState(int newValue)
{
    pCurrentState->setValue(newValue);
    pCurrentState->notify();
}

void BLE_Manager::changeValue_Accelerometer_value(SensorData *sensorData)
{
    float mag = sensorData->getLastAvaiableData();

    float2bytes data;
    data.f = mag;

    pAccelerometerData->setValue(data.b, sizeof(data));
    pAccelerometerData->indicate();
}
