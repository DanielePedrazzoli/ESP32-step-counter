#pragma once

#ifndef _MPU6050_6AXIS_MOTIONAPPS612_H_
#include <MPU6050_6Axis_MotionApps612.h>
#endif

#define MAX_SAMPLE_NUMBER 100

#include <Arduino.h>
#include <dsps_fir.h>

#include "Costants.h"
#include "esp_dsp.h"
#include "ekf_imu13states.h"
#include "kalmanFilter.h"

class SensorData
{
public:
    SensorData();
    ~SensorData();

    void init();

    void computeMagnitude(int16_t, int16_t, int16_t);
    void computeMagnitude(VectorInt16 *);

    float getLastAvaiableData();
    float getLastAvaiableData_net();

    // Varibili par array circolare
    int last_mag_index = 0;

    float mag[MAX_SAMPLE_NUMBER];
    float mag_net[MAX_SAMPLE_NUMBER];
    float mag_avg;
    float filtred_value;
    SimpleKalmanFilter *kalmanFilter;

private:
    // Filter variable
    // Dato che tutti i valori di questa classe arrivano dallo stesso sensore
    // (indipendentemente che esso sia l'acelerometro o il giroscopio)
    // allora dovranno essere soggetti allo stesso filtro
    // Ha senso quindi definire la variabile di filtraggio come interna
    // per ridurre il codice esterno
    fir_s16_t filterStructure;
    int16_t delay[MAX_SAMPLE_NUMBER];
    uint16_t history[MAX_SAMPLE_NUMBER];

    void computeAverage();
    void applyFilter();
};
