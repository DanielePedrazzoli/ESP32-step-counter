#pragma once

#include <Arduino.h>
#include <dsps_fir.h>

#include "Costants.h"

class SensorData
{
public:
    SensorData();
    ~SensorData();

    int16_t lastDataOf(AXES);
    int16_t startFilteringAxes(AXES);

    void initFilter();
    int addValue(AXES, int16_t);

    // Varibili par array circolare
    int last_x_index = 0;
    int last_y_index = 0;
    int last_z_index = 0;

    // // Filtered Value
    // int16_t x_values_filtered[N_VALUE_PER_READING];
    // int16_t y_values_filtered[N_VALUE_PER_READING];
    // int16_t z_values_filtered[N_VALUE_PER_READING];
    // Raw Value
    int16_t x_values[SAMPLE_FILTER_TAP_NUM];
    int16_t y_values[SAMPLE_FILTER_TAP_NUM];
    int16_t z_values[SAMPLE_FILTER_TAP_NUM];

    int16_t x_filtered_value;
    int16_t y_filtered_value;
    int16_t z_filtered_value;

private:
    // Filter variable
    // Dato che tutti i valori di questa classe arrivano dallo stesso sensore
    // (indipendentemente che esso sia l'acelerometro o il giroscopio)
    // allora dovranno essere soggetti allo stesso filtro
    // Ha senso quindi definire la variabile di filtraggio come interna
    // per ridurre il codice esterno
    fir_s16_t filterStructure;
    int16_t delay[SAMPLE_FILTER_TAP_NUM];
    uint16_t lastIndex;
    uint16_t history[SAMPLE_FILTER_TAP_NUM];
};
