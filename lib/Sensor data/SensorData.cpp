#include "sensorData.h"
#include "Costants.h"

// Implementazione del costruttore
SensorData::SensorData() {}

// Implementazione del distruttore
SensorData::~SensorData() {}

int16_t SensorData::lastDataOf(AXES axes)
{
    switch (axes)
    {
    case AXES::X:
        return x_values[last_x_index];
        break;

    case AXES::Y:
        return y_values[last_y_index];
        break;

    case AXES::Z:
        return z_values[last_z_index];
        break;

    default:
        return 0;
    }
}

int16_t SensorData::startFilteringAxes(AXES axes)
{

    int16_t *pointer;
    switch (axes)
    {
    case AXES::X:
        pointer = x_values;
        break;

    case AXES::Y:
        pointer = y_values;
        break;

    case AXES::Z:
        pointer = z_values;
        break;
    }

    int32_t acc = 0;
    int index = lastIndex, i;
    for (i = 0; i < SAMPLE_FILTER_TAP_NUM; ++i)
    {
        index = index != 0 ? index - 1 : SAMPLE_FILTER_TAP_NUM - 1;
        acc += (long long)pointer[index] * ACC_FILTER_TAPS[i];
    };

    switch (axes)
    {
    case AXES::X:
        x_filtered_value = (int16_t)(acc >> 16);
        break;

    case AXES::Y:
        y_filtered_value = (int16_t)(acc >> 16);
        break;

    case AXES::Z:
        z_filtered_value = (int16_t)(acc >> 16);
        break;
    }

    return 0;
}

void SensorData::initFilter()
{

    // esp_err_t err = dsps_fird_init_s16(&filterStructure, filter_taps, delay, SAMPLEFILTER_TAP_NUM, 1, 0, 0);
    // if (err != ESP_OK)
    // {
    // }
    x_filtered_value = 0;
    y_filtered_value = 0;
    z_filtered_value = 0;
    lastIndex = 0;
    int i;
    for (i = 0; i < SAMPLE_FILTER_TAP_NUM; ++i)
    {
        x_values[i] = 0;
        y_values[i] = 0;
        z_values[i] = 0;
    }
}

int SensorData::addValue(AXES axes, int16_t value)
{

    switch (axes)
    {
    case AXES::X:
        x_values[last_x_index++] = value;
        if (last_x_index >= SAMPLE_FILTER_TAP_NUM)
        {
            last_x_index = 0;
        }
        break;

    case AXES::Y:
        y_values[last_y_index++] = value;
        if (last_y_index >= SAMPLE_FILTER_TAP_NUM)
        {
            last_y_index = 0;
        }
        break;

    case AXES::Z:
        z_values[last_z_index++] = value;
        if (last_z_index >= SAMPLE_FILTER_TAP_NUM)
        {
            last_z_index = 0;
        }
        break;

    default:
        return -1;
    }

    return 0;
}
