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
    for (i = 0; i < SAMPLE_FILTER_TAP_NUM; ++i)
    {
        mag[i] = 0;
        mag_net[i] = 0;
    }
}

/**
 * @brief Effettua un calcolo del modulo e aggiorna in automatico il valore di media
 *
 * @param x valore asse x
 * @param y valore asse y
 * @param z valore asse z
 */
void SensorData::computeMagnitude(int16_t x, int16_t y, int16_t z)
{
    double magValue = sqrt(x * x + y * y + z * z);

    mag[last_mag_index] = magValue;
    computeAverage();
    mag_net[last_mag_index] = magValue - mag_avg;

    // incremento indice e controllo per la gestione del buffer circolare
    last_mag_index++;
    if (last_mag_index >= SAMPLE_FILTER_TAP_NUM)
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
    double temp = 0;
    for (uint8_t i = 0; i < SAMPLE_FILTER_TAP_NUM; i++)
    {
        temp += mag[i];
    }
    mag_avg = temp / SAMPLE_FILTER_TAP_NUM;
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
    double acc = 0;
    int index = last_mag_index;
    for (int i = 0; i < SAMPLE_FILTER_TAP_NUM; ++i)
    {
        index = index != 0 ? index - 1 : SAMPLE_FILTER_TAP_NUM - 1;
        acc += (long long)mag_net[index] * FILTER_TAPS[i];
    };
    filtred_value = acc;
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