#ifndef _SENSOR_DATA_
#define _SENSOR_DATA_

#include <MPU6050_6Axis_MotionApps612.h>
#include <Arduino.h>
#include "SampleFilter.h"
#include "esp_dsp.h"
#include "SampleFilter.h"

#define SAMPLE_HISTORY_LENGTH SAMPLEFILTER_TAP_NUM

#define MILLIS_IN_SECOND 1000

enum MotionState
{
    unknow = 0,
    slowWalk = 1,
    walk = 2,
    run = 3,
};

struct Step
{
    int start;
    int end;
    int peek;
    MotionState stepType;
};

enum StepWindowState
{
    NoOperation,
    StepStarted,
    peekDetected,

};

class SensorData
{
public:
    SensorData();
    ~SensorData();

    void init();
    bool pushValue(VectorInt16 *);
    float getLastAvaiableData();

    int last_value_index = 0;
    float values[SAMPLEFILTER_TAP_NUM] = {0};

    bool isFisrtValue = true;
    unsigned long numbersOfValue = 0;
    unsigned long totalValueRead = 0;
    Step mosteRecentStep;

private:
    bool computevalue();

    SampleFilter filterStruct;
    int samplesFomrLastStep = 0;
    StepWindowState currentStepState = StepWindowState::NoOperation;
    Step currentStep;

    int lastPeekValue = 0;
    int lowerThreshold = 0;

    int run_H_Th = 24560; // ~ 1.5g
    int run_L_Th = 19660; // ~ 1.2g

    int walk_H_Th = 9000; // ~ 0.55g
    int walk_L_Th = 6554; // ~ 0.40g

    int slowWalk_H_Th = 4900; // ~ 0.3g
    int slowWalk_L_Th = 3280; // ~ 0.2g
};

#endif