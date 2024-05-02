

#include "BLEManager.h"
#include "Costants.h"

const char *const BLE_Manager::DEVICE_NAME = "ESP 32 motion sensor";

const char *const BLE_Manager::SENSOR_UUID = "130db0c9-e717-45ba-8a99-21d03144f056";
const char *const BLE_Manager::DATA_UUID = "2f766cb0-804b-4779-ba00-80a3f08cc70e";
const char *const BLE_Manager::ACCELEROMETER_UUID = "7c15c5fb-f9a8-41d2-885f-07db1827370f";
const char *const BLE_Manager::GYROSCOPE_UUID = "494a449d-2d06-4050-a8b4-3abd2c0e5f63";
const char *const BLE_Manager::STEP_UUID = "3595f562-1fa2-45d1-9f77-81d9e70aebca";
const char *const BLE_Manager::STATE_UUID = "c8754d82-859f-4724-b6e5-a2e7cd7f7479";

// Implementazione del costruttore
BLE_Manager::BLE_Manager(/* args */)
{
}

// Implementazione del distruttore
BLE_Manager::~BLE_Manager()
{
}

// Implementazione del metodo init()
int BLE_Manager::init()
{
    NimBLEDevice::init(DEVICE_NAME);
    pServer = NimBLEDevice::createServer();

    /* Inizializzazione service sensori */
    pSensorService = pServer->createService(BLEUUID(SENSOR_UUID));
    pAccelerometerData = pSensorService->createCharacteristic(ACCELEROMETER_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pGyroscopeData = pSensorService->createCharacteristic(GYROSCOPE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pSensorService->start();

    /* Inizializzazione service dati */
    pDataService = pServer->createService(BLEUUID(DATA_UUID));
    pStepCout = pDataService->createCharacteristic(STEP_UUID, NIMBLE_PROPERTY::READ);
    pCurrentState = pDataService->createCharacteristic(STATE_UUID, NIMBLE_PROPERTY::READ);
    pDataService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SENSOR_UUID);
    pAdvertising->addServiceUUID(DATA_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();
    return 1;
}

void BLE_Manager::changeValue_step(int newValue)
{
    pStepCout->setValue(newValue);
    pStepCout->notify();
}

void BLE_Manager::changeValue_state(int newValue)
{
    pCurrentState->setValue(newValue);
    pCurrentState->notify();
}

void BLE_Manager::split16to8(int16_t *input, uint8_t *output, uint8_t size)
{
    for (int i = 0; i < size; i++)
    {
        output[i * 2] = input[i] >> 8;
        output[i * 2 + 1] = input[i] & 0xFF;
    }
}
void splitData(uint8_t *buffer, uint16_t start, int16_t value)
{
    buffer[start] = value >> 8;
    buffer[start + 1] = value & 0xFF;
}

void BLE_Manager::changeValue_Accelerometer_value(SensorData *sensorData)
{
    int16_t value_x = sensorData->x_filtered_value;
    int16_t value_y = sensorData->z_values[sensorData->last_z_index];
    int16_t value_z = sensorData->z_filtered_value;
    uint8_t data[6];
    data[0] = value_x >> 8;
    data[1] = value_x & 0xFF;
    data[2] = value_y >> 8;
    data[3] = value_y & 0xFF;
    data[4] = value_z >> 8;
    data[5] = value_z & 0xFF;
    pAccelerometerData->setValue(data, sizeof(data));
    pAccelerometerData->indicate();
}

void BLE_Manager::changeValue_Gyroscope_value(SensorData *sensorData)
{
    int16_t value_x = sensorData->lastDataOf(AXES::X);
    uint16_t value_y = sensorData->lastDataOf(AXES::Y);
    uint16_t value_z = sensorData->lastDataOf(AXES::Z);
    uint8_t data[6];
    data[0] = value_x >> 8;
    data[1] = value_x & 0xFF;
    data[2] = value_y >> 8;
    data[3] = value_y & 0xFF;
    data[4] = value_z >> 8;
    data[5] = value_z & 0xFF;
    pGyroscopeData->setValue(data, sizeof(data));
    pGyroscopeData->indicate();
}
