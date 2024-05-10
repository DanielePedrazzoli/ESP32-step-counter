#include "MotionSensor.h"
// #ifndef _BLE_MANAGER_
// #include "BLEManager.h"
// #endif
// extern BLE_Manager bleManager;
void MotionSensor::init(BLE_Manager *m)
{
    manager = m;
}

MotionSensor::MotionSensor()
{
    for (int i = 0; i < STEP_HISTORY_NUMBER; i++)
    {
        step_history[i] = 0;
    }
}

void MotionSensor::addStep()
{
    step++;
    updateHistory();
    updateState();
    manager->changeStepValue(step);
    manager->changeMotionState(currentMotionState);
}

void MotionSensor::updateHistory()
{
    step_history[history_index++] = millis();
    if (history_index >= STEP_HISTORY_NUMBER)
    {
        history_index = 0;
    }

    long sum = 0;
    for (int i = 0; i < STEP_HISTORY_NUMBER; i++)
    {
        sum += step_history[i];
    }

    long period = sum / STEP_HISTORY_NUMBER;

    if (period != 0)
    {
        step_freq = 1 / period;
    }
}

void MotionSensor::updateState()
{
    if (step_freq < SLOW_WALKING_TH)
    {
        currentMotionState = motionStates::STANDING;
        return;
    }

    if (step_freq < WALKING_TH)
    {
        currentMotionState = motionStates::SLOW_WALKING;
        return;
    }

    if (step_freq < FAST_WALKING_TH)
    {
        currentMotionState = motionStates::WALKING;
        return;
    }

    if (step_freq < RUNNING_TH)
    {
        currentMotionState = motionStates::FAST_WALKING;
        return;
    }

    currentMotionState = motionStates::RUNNING;
}