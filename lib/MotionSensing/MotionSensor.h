

#ifndef _MOTION_SENSING_
#define _MOTION_SENSING_
#include <Arduino.h>
#include "BLEManager.h"

#define STEP_HISTORY_NUMBER 5

#define AVG_SLOW_WALK_SPEED 1.25f // m/s --> 4 km/h
#define AVG_WALK_SPEED 1.38f      // m/s --> 5 km/h
#define AVG_RUN_SPEED 1.66f       // m/s --> 7 km/h

class MotionSensor
{
private:
    float step_freq;
    uint8_t history_index;
    BLE_Manager *manager;

    Step stepHistory[STEP_HISTORY_NUMBER];
    char stepHistoryIndex = 0;
    double distanceTravelled = 0;

public:
    MotionSensor();
    void init(BLE_Manager *);
    void addStep(Step *);

    void sampleStep(SensorData *);

    int step;
};

#endif
