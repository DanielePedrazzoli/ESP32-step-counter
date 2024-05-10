#include "MotionSensor.h"

/**
 * @brief Funzione di inizializzazione
 *
 * @param m puntatore al Manager BLE per la comunicazione BLE
 */
void MotionSensor::init(BLE_Manager *m)
{
    manager = m;
}

/**
 * @brief Costruttore della classe
 *
 */
MotionSensor::MotionSensor()
{
    for (int i = 0; i < STEP_HISTORY_NUMBER; i++)
    {
        step_history[i] = 0;
    }
}

/**
 * @brief incrementa il numero degli step salavti e aggiorna lo stato di movimento.
 * Notifica anche alla libreria BLE che il valore è stato modificato
 *
 */
void MotionSensor::addStep()
{
    step++;
    updateHistory();
    updateState();
    manager->changeStepValue(step);
    manager->changeMotionState(currentMotionState);
}

/**
 * @brief Aggiorna gli ultimi valori in modo da osservare quando tempo
 * è passato tra uno step rilevato ed un altro. Questo permette di ottenere
 * lo stato di movimento
 *
 */
void MotionSensor::updateHistory()
{
    step_history[history_index++] = millis();

    // buffer circolare
    if (history_index >= STEP_HISTORY_NUMBER)
    {
        history_index = 0;
    }

    // supponendo di avere 5 valori x1,x2,x3,x4,x5.
    // Effettuare la media tra la differenza di questi valori equivale a
    // calcolare (x5 - x1) / 5
    // Naturalmente questo va applicto al concetto di buffer circolare
    uint8_t mostRecentValueIndex;
    uint8_t lastRencetValueIndex;

    if (history_index == 0)
    {
        // Se l'indice attuale è nullo, il valore più recente aggiunto si trova
        // quindi come ultima posizione dell'array
        // Allo stesso tempo significa che il valore più vecchio si trova all'indice 0
        mostRecentValueIndex = step_history[STEP_HISTORY_NUMBER - 1];
        lastRencetValueIndex = step_history[0];
    }
    else
    {
        // Se l'indice attuale è diverso da zero, il valore più recente aggiunto si trova sempre
        // all'indice precdente del puntatore attuale. Questo poiché ogni volta che aggiunge un
        // valore incremento il contatore. Di conseguenza l'ultimo valore si trova sempre
        // all'indice del puntatore attuale
        mostRecentValueIndex = step_history[history_index - 1];
        lastRencetValueIndex = step_history[history_index];
    }

    long diff = (mostRecentValueIndex - lastRencetValueIndex) / STEP_HISTORY_NUMBER;

    // Se la differenza è nulla o inferiore a 0 allora la ignoro.
    // Probabilmente sono in una fase iniziale e i valori devono
    // ancora essere riempiti
    if (diff == 0 || diff < 0)
        return;

    // Se considero la differenza appena ottenuta come periodo del passo allora posso calcolare
    // la sua frequenza effettuando il suo inverso

    step_freq = 1 / diff;
}

/**
 * @brief Aggiorna lo stato attuale di movimento in base a delle soglie
 * di frequenza di passi prefissate
 *
 */
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