#include "BLEManager.h"

const char *const BLE_Manager::DEVICE_NAME = "ESP 32 motion sensor";

const char *const BLE_Manager::DATA_UUID = "2f766cb0-804b-4779-ba00-80a3f08cc70e";
const char *const BLE_Manager::STEP_UUID = "3595f562-1fa2-45d1-9f77-81d9e70aebca";
const char *const BLE_Manager::FREQUENCY_UUID = "c8754d82-859f-4724-b6e5-a2e7cd7f7479";
const char *const BLE_Manager::DISTANCE_UUID = "6ae11cb8-7db8-44bd-aa76-2e5b1df76531";

#if DEBUG_MODE
const char *const BLE_Manager::SENSOR_UUID = "130db0c9-e717-45ba-8a99-21d03144f056";
const char *const BLE_Manager::ACCELEROMETER_UUID = "7c15c5fb-f9a8-41d2-885f-07db1827370f";
#endif

/**
 * @brief Costruttore classe
 *
 */
BLE_Manager::BLE_Manager() {}

/**
 * @brief Funzione di inzializzaione. crea tutti i service relativi ai dati dei sensori
 *
 * @return int codice di errore. Se è diverso da 0 è in errore
 */
int BLE_Manager::init()
{
    NimBLEDevice::init(DEVICE_NAME);
    pServer = NimBLEDevice::createServer();

    /* Inizializzazione service sensori */

#if DEBUG_MODE
    pSensorService = pServer->createService(BLEUUID(SENSOR_UUID));
    pAccelerometerData = pSensorService->createCharacteristic(ACCELEROMETER_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pSensorService->start();
#endif

    /* Inizializzazione service dati */
    pDataService = pServer->createService(BLEUUID(DATA_UUID));
    pStepCout = pDataService->createCharacteristic(STEP_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pStepFrequency = pDataService->createCharacteristic(FREQUENCY_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pDistanceTravelled = pDataService->createCharacteristic(DISTANCE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    pDataService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
#if DEBUG_MODE
    pAdvertising->addServiceUUID(SENSOR_UUID);
#endif
    pAdvertising->addServiceUUID(DATA_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();
    return 0;
}

/**
 * @brief Modifica il valore dei step e notifica gli altri dispositivi
 *
 * @param newValue
 */
void BLE_Manager::changeStepValue(int newValue)
{
    pStepCout->setValue(newValue);
    pStepCout->notify();
}

/**
 * @brief Modifica il valore di stato e notifica gli altri dispositivi
 *
 * @param newValue
 */
void BLE_Manager::updateAnalisysData(float freq, float distance)
{
    pStepFrequency->setValue(freq);
    pStepFrequency->notify();

    pDistanceTravelled->setValue(distance);
    pDistanceTravelled->notify();
}

#if DEBUG_MODE
/**
 * @brief Modifica il valore dell'accelerometro e notifica i dispositivi.
 * Questa funzione è pensata per essere usata come test poiché a tutti gli
 * effetti invia uno stream di dati che non sarebbe molto coerente con il BLE
 *
 * @param sensorData Il valore dell'acceleromentro
 */
void BLE_Manager::changeValue_Accelerometer_value(SensorData *sensorData)
{
    float mag = sensorData->getLastAvaiableData();

    float2bytes data;
    data.f = mag;

    pAccelerometerData->setValue(data.b, sizeof(data));
    pAccelerometerData->indicate();
}
#endif