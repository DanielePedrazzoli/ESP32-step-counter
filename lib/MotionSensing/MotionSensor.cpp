#include "MotionSensor.h"

/**
 * @brief Funzione di inizializzazione
 *
 * @param m puntatore al Manager BLE per la comunicazione BLE
 */
void MotionSensor::init(BLE_Manager *m)
{
    manager = m;

    for (int i = 0; i < STEP_HISTORY_NUMBER; i++)
    {
        stepHistory[i] = {0, 0, 0, MotionState::unknow};
    }
}

/**
 * @brief Costruttore della classe
 *
 */
MotionSensor::MotionSensor()
{
}

/**
 * @brief incrementa il numero degli step salavti e aggiorna lo stato di movimento.
 * Notifica anche alla libreria BLE che il valore è stato modificato
 *
 */
void MotionSensor::addStep(Step *lastAddedStep)
{
    step++;
    manager->changeStepValue(step);

    // Provo a stimare una distanza percorsa in base alla durata del passo e all'accelerazione
    // ottenuta
    // Data la durata del passo la stima avviene dividendo per la frequenza di campionamento
    // e moltiplicando per la velocità media percorribile dato quello stato di movimento
    double stepLenght = lastAddedStep->end - lastAddedStep->start;
    switch (lastAddedStep->stepType)
    {
    case MotionState::slowWalk:
        distanceTravelled += (double)(stepLenght / 100) * AVG_SLOW_WALK_SPEED;
        break;

    case MotionState::walk:
        distanceTravelled += (double)(stepLenght / 100) * AVG_WALK_SPEED;
        break;

    case MotionState::run:
        distanceTravelled += (double)(stepLenght / 100) * AVG_RUN_SPEED;
        break;

    default:
        break;
    }
    Serial.print("step lenght: ");
    Serial.print(stepLenght);
    Serial.print("\t");
    Serial.print("current distance: ");
    Serial.print(distanceTravelled);
    Serial.print("\t");
}

/**
 * @brief Estra l'ultimo step rilevato e lo prede in considerazione per il rilevamento dello stato di movimento
 *
 * @param sensor
 */
void MotionSensor::sampleStep(SensorData *sensor)
{
    stepHistory[history_index] = sensor->mosteRecentStep;
    // Calcolo la media della distanza dei passi (quindi dei picchi dei passi)
    // Questo mi permette di calcolare a quanti passi al secondi sto andando
    double sum = 0;
    int motionStateIndex = 0;
    for (int i = 0; i < STEP_HISTORY_NUMBER - 1; i++)
    {
        int currentPeek = stepHistory[(history_index - i + STEP_HISTORY_NUMBER) % STEP_HISTORY_NUMBER].peek;
        int prevPeek = stepHistory[(history_index - i - 1 + STEP_HISTORY_NUMBER) % STEP_HISTORY_NUMBER].peek;
        sum += currentPeek - prevPeek;
    }

    // questo valore rappresenta la media della distanza tra gli ultimi 5 picchi (quindi 5 step)
    float avgPeekDistance = (float)(sum / (STEP_HISTORY_NUMBER - 1));

    // Di conseguenza, sapendo la frequenza di campionamto (200Hz) posso so che
    // effettuare un passo di metto mediamente:
    //
    // avgPeekDistance [campioni]
    // ------------------------- = v [passi al secondo]
    /// 200 [campioni/secondo]

    step_freq = avgPeekDistance / 200;

    Serial.print("current freq: ");
    Serial.println(step_freq);
    manager->updateAnalisysData(step_freq, distanceTravelled);
    history_index = (history_index + 1) % STEP_HISTORY_NUMBER;
}
