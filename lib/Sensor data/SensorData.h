#pragma once

#include <Arduino.h>
#include <dsps_fir.h>

#include "Costants.h"

class SensorData
{
public:
    SensorData();
    ~SensorData();

    void init();

    void computeMagnitude(int16_t, int16_t, int16_t);

    float getLastAvaiableData();
    float getLastAvaiableData_net();

    // Varibili par array circolare
    int last_mag_index = 0;

    float mag[SAMPLE_FILTER_TAP_NUM];
    float mag_net[SAMPLE_FILTER_TAP_NUM];
    float mag_avg;
    float filtred_value;

private:
    // Filter variable
    // Dato che tutti i valori di questa classe arrivano dallo stesso sensore
    // (indipendentemente che esso sia l'acelerometro o il giroscopio)
    // allora dovranno essere soggetti allo stesso filtro
    // Ha senso quindi definire la variabile di filtraggio come interna
    // per ridurre il codice esterno
    fir_s16_t filterStructure;
    int16_t delay[SAMPLE_FILTER_TAP_NUM];
    uint16_t history[SAMPLE_FILTER_TAP_NUM];

    void computeAverage();
    void applyFilter();
};
