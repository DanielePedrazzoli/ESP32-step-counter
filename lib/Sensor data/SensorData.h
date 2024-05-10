#ifndef _SENSOR_DATA_
#define _SENSOR_DATA_

#include <MPU6050_6Axis_MotionApps612.h>
#include <Arduino.h>
#include "Costants.h"
#include "SampleFilter.h"
#include "esp_dsp.h"
#include "SampleFilter.h"

#define MAX_VALUE_NUMBER SAMPLEFILTER_TAP_NUM

class SensorData
{
public:
    SensorData();
    ~SensorData();

    void init();
    bool addValue(VectorInt16 *);
    float getLastAvaiableData();

    int last_value_index = 0;
    float values[SAMPLEFILTER_TAP_NUM];

private:
    bool computevalue();

    // SimpleKalmanFilter *kalmanFilter;

    SampleFilter filterStruct;

    bool isAscending = false;
    bool alreadyOverTreshold = false;

    int threshold = 6553;
    unsigned long lastUnderThreshold;
    int samplesFomrLastStep = 0;
};

#endif