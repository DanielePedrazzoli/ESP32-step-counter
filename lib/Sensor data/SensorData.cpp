#include "sensorData.h"

// Implementazione del costruttore
SensorData::SensorData() {}

// Implementazione del distruttore
SensorData::~SensorData() {}

/**
 * @brief Funzione di inizializzaione valori della classe
 */
void SensorData::init()
{
    // kalmanFilter = new SimpleKalmanFilter(500, 1000, 0.01);

    // int i;
    // for (i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
    // {
    //     filterStruct.history[i] = 0;
    // }
    // filterStruct.last_index = 0;
    SampleFilter_init(&filterStruct);
}

/**
 * @brief Effettua un calcolo del modulo e aggiorna in automatico il valore di media
 *
 * @param v vettore di accelerazione
 */
bool SensorData::addValue(VectorInt16 *v)
{
    double magValue = v->getMagnitude();

    SampleFilter_put(&filterStruct, magValue);
    values[last_value_index++] = SampleFilter_get(&filterStruct);
    if (last_value_index >= MAX_VALUE_NUMBER)
    {
        last_value_index = 0;
    }

    // return false;

    return computevalue();
}

float SensorData::getLastAvaiableData()
{
    return values[last_value_index];
}

bool SensorData::computevalue()
{
    int secondToLast;
    int last;
    int current = values[last_value_index];
    samplesFomrLastStep++;

    if (!alreadyOverTreshold && current > threshold)
    {
        alreadyOverTreshold = true;

        // controllo se sono stato al di sotto della threshold per un quantitativo
        // di tempo ragionevole
        // Questo permette di eliminare alcuni falsi passi che verrebbero rilevati
        if (samplesFomrLastStep >= 70)
        {
            samplesFomrLastStep = 0;
            return true;
        }
        return false;
    }

    if (alreadyOverTreshold && current < threshold)
    {
        samplesFomrLastStep = 0;
        alreadyOverTreshold = false;
    }

    return false;
}