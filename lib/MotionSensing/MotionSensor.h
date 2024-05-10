

#ifndef _MOTION_SENSING_
#define _MOTION_SENSING_
#include <Arduino.h>
#include "BLEManager.h"

#define STEP_HISTORY_NUMBER 5

#define SLOW_WALKING_TH 1
#define WALKING_TH 2
#define FAST_WALKING_TH 3
#define RUNNING_TH 4

enum motionStates
{
    STANDING = 0,
    SLOW_WALKING = 1,
    WALKING = 2,
    FAST_WALKING = 3,
    RUNNING = 4,
};

class MotionSensor
{
private:
    void updateHistory();
    void updateState();

    unsigned long step_history[STEP_HISTORY_NUMBER];
    float step_freq;
    uint8_t history_index;
    BLE_Manager *manager;

public:
    MotionSensor();
    void init(BLE_Manager *);
    void addStep();

    int step;
    motionStates currentMotionState = motionStates::STANDING;
};

#endif
