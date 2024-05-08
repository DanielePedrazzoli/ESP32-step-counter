#include "sensorData.h"
#include "Costants.h"

// Implementazione del costruttore
SensorData::SensorData() {}

// Implementazione del distruttore
SensorData::~SensorData() {}

/**
 * @brief Funzione di inizializzaione valori della classe
 */
void SensorData::init()
{

    mag_avg = 0;
    int i;
    for (i = 0; i < MAX_SAMPLE_NUMBER; ++i)
    {
        mag[i] = 0;
        mag_net[i] = 0;
    }

    kalmanFilter = new SimpleKalmanFilter(500, 1000, 0.01);
}

/**
 * @brief Effettua un calcolo del modulo e aggiorna in automatico il valore di media
 *
 * @param accelVector
 */
void SensorData::computeMagnitude(VectorInt16 *accelVector)
{
    double magValue = accelVector->getMagnitude();

    mag_net[last_mag_index] = magValue;
    // computeAverage();
    // mag_net[last_mag_index] = magValue - mag_avg;

    // incremento indice e controllo per la gestione del buffer circolare
    last_mag_index++;
    if (last_mag_index >= MAX_SAMPLE_NUMBER)
    {
        last_mag_index = 0;
    }
    applyFilter();
}

/**
 * @brief Esegue una nuova computazione della media dei valori con quelli attualmente presenti dentro l'array `mag`
 *
 */
void SensorData::computeAverage()
{

    double sum = 0;
    for (uint8_t i = 0; i < MAX_SAMPLE_NUMBER; i++)
    {
        sum += mag[i];
    }
    mag_avg = sum / MAX_SAMPLE_NUMBER;
}

float SensorData::getLastAvaiableData()
{
    return mag[last_mag_index];
}

float SensorData::getLastAvaiableData_net()
{
    return mag_net[last_mag_index];
}

void SensorData::applyFilter()
{
    // filtred_value = mag_net[last_mag_index];
    // double acc = 0;
    // int index = last_mag_index;
    // for (int i = 0; i < SAMPLE_FILTER_TAP_NUM; ++i)
    // {
    //     index = index != 0 ? index - 1 : SAMPLE_FILTER_TAP_NUM - 1;
    //     acc += (long long)mag_net[index] * FILTER_TAPS[i];
    // };
    // filtred_value = acc;

    filtred_value = kalmanFilter->updateEstimate(mag_net[last_mag_index]);
}

// int16_t SensorData::startFilteringAxes(AXES axes)
// {

//     int16_t *pointer;
//     switch (axes)
//     {
//     case AXES::X:
//         pointer = x_values;
//         break;

//     case AXES::Y:
//         pointer = y_values;
//         break;

//     case AXES::Z:
//         pointer = z_values;
//         break;
//     }

//     int32_t acc = 0;
//     int index = lastIndex, i;
//     for (i = 0; i < SAMPLE_FILTER_TAP_NUM; ++i)
//     {
//         index = index != 0 ? index - 1 : SAMPLE_FILTER_TAP_NUM - 1;
//         acc += (long long)pointer[index] * ACC_FILTER_TAPS[i];
//     };

//     switch (axes)
//     {
//     case AXES::X:
//         x_filtered_value = (int16_t)(acc >> 16);
//         break;

//     case AXES::Y:
//         y_filtered_value = (int16_t)(acc >> 16);
//         break;

//     case AXES::Z:
//         z_filtered_value = (int16_t)(acc >> 16);
//         break;
//     }

//     return 0;
// }