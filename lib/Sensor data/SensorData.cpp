#include "sensorData.h"

// Implementazione del costruttore
SensorData::SensorData() {}

// Implementazione del distruttore
SensorData::~SensorData() {}

/**
 * @brief Funzione di inizializzaione valori della classe
 */
void SensorData::init()
{
    SampleFilter_init(&filterStruct);

    for (int i = 0; i < SAMPLE_HISTORY_LENGTH; i++)
    {
        values[i] = 0;
    }
}

/**
 * @brief Effettua un calcolo del modulo e aggiorna in automatico il valore di media
 *
 * @param v vettore di accelerazione
 */
bool SensorData::pushValue(VectorInt16 *v)
{
    double magValue = v->getMagnitude();

    // filtro valore
    SampleFilter_put(&filterStruct, magValue);

    // push e buffer circolare
    values[last_value_index] = SampleFilter_get(&filterStruct);
    numbersOfValue++;
    last_value_index = (last_value_index + 1) % SAMPLE_HISTORY_LENGTH;

    totalValueRead++;

    return computevalue();
}

/**
 * @brief Ritorna l'ultimo valore salvato nel buffer circolare
 *
 * @return float
 */
float SensorData::getLastAvaiableData()
{
    if (last_value_index - 1 < 0)
    {
        return values[SAMPLE_HISTORY_LENGTH - 1];
    }
    return values[last_value_index - 1];
}

/**
 * @brief Esegue un controllo per verifica se lo step è vero oppure no.
 * Controlla se gli step rilevati sono almento ad un tot di sample l'uno dall'altro.
 * Questo dovrebbe permettere di evitare di contare dei falsi step dovuti a delle oscillazioni
 *
 * Durante questa fase determino anche la lungehzza dello step. Per farlo è necessario che
 * rilevato l'inizio, il picco e la fine dello step.
 *
 * - L'inizio è rilevato con una ascesa di valori
 * - Il picco come un picco ma solo se supera la upperThreshold
 * - La fine come una nuova ascesa di valori una volta che sia è andati sotto la lowerThreshold
 *
 * @return true se lo step è considerato valido.
 * @return false se lo step non è valido
 */
bool SensorData::computevalue()
{
    int current = values[(last_value_index - 1 + SAMPLE_HISTORY_LENGTH) % SAMPLE_HISTORY_LENGTH];
    int last = values[(last_value_index - 2 + SAMPLE_HISTORY_LENGTH) % SAMPLE_HISTORY_LENGTH];
    int secondToLast = values[(last_value_index - 3 + SAMPLE_HISTORY_LENGTH) % SAMPLE_HISTORY_LENGTH];

    // Se ho rilevato un picco o un inizio di passo tengo presente quanti valori sto considerando
    // un passo difficeilmente è più lungo di un secondo. Di conseguenza
    // se come analisi del passo corrente ci sto mettendo più di 0.75 secondi allora annullo tutto.
    // probabilmente era un errore.
    // Altirimenti se ho rtilevato un picco allora posso controllare in abse alla media umana quando
    // potrebbe essere lungo al massimo il passo. In particolare
    // camminata lenta          = 60-80 step/m ---> 1-1.3 s  tep/s ---> max 120
    // camminata                = 80-100 step/m --> 1.3-1.6 step/s ---> max 100
    // camminata veloce / corsa = 100-120 step/m -> 1.6-2  step/s  ---> max 75
    if (currentStepState != StepWindowState::NoOperation)
    {
        int analysisLength = totalValueRead - currentStep.start;
        int MaxAalysisLength = 150;

        if (currentStep.stepType == MotionState::run)
        {
            MaxAalysisLength = 100;
        }
        else if (currentStep.stepType == MotionState::walk)
        {
            MaxAalysisLength = 120;
        }
        else if (currentStep.stepType == MotionState::slowWalk)
        {
            MaxAalysisLength = 130;
        }

        if (analysisLength > MaxAalysisLength)
        {
            Serial.println("Step aborted, Max lenght");
            currentStepState = StepWindowState::NoOperation;
            currentStep = {0, 0, 0, MotionState::unknow};
            lowerThreshold = 0;
        }
    }

    switch (currentStepState)
    {
    case StepWindowState::NoOperation:

        // Se identifico una salita allora è possibile che ci sia un passo
        if (current > slowWalk_L_Th && secondToLast < last && last < current)
        {
            currentStepState = StepWindowState::StepStarted;
            Serial.print("Step Started\tcurrent value: ");
            Serial.println(current);
            currentStep.start = totalValueRead - 2;
        }
        break;

    case StepWindowState::StepStarted:

        // Identifico se è presente un picco (rappresentato da last)
        if (last > slowWalk_H_Th && last > secondToLast && last > current)
        {
            // i picchi devono essere distanziati sufficentemente

            // int lastPeekIndex = stepHistory[(stepHistoryIndex + STEP_HISTORY_NUMBER - 1) % STEP_HISTORY_NUMBER].peek;
            // if (totalValueRead - lastPeekIndex < 25)
            //     break;

            currentStep.peek = totalValueRead;
            Serial.print("Peek detected:\t");
            Serial.println(last);

            lastPeekValue = last;
            // ora che sono nel picco posso guardare il valore del picco e definire la soglia
            // La soglia inferiore di modifica dinamicamente in base al picco rilevato
            // Se il segnale un picco elevato, allora è più probabile che stia correndo e quindi
            // imposto la soglia inferiore più in alto. Allo stesso tempo se una persona sta camminando
            // non ha senso avere una soglia inferiore molto alta poicé si rischia di non arrivarci mai
            if (last >= run_H_Th)
            {
                lowerThreshold = run_L_Th;
                currentStep.stepType = MotionState::run;
                Serial.println("peek run");
            }
            else if (last >= walk_H_Th)
            {
                lowerThreshold = walk_L_Th;
                currentStep.stepType = MotionState::walk;
                Serial.println("peek walk");
            }
            else if (last >= slowWalk_H_Th)
            {
                lowerThreshold = slowWalk_L_Th;
                currentStep.stepType = MotionState::slowWalk;
                Serial.println("peek slow");
            }

            currentStepState = StepWindowState::peekDetected;
        }

        break;

    case StepWindowState::peekDetected:
        // Se ho già identifica un picco, allora la soglia è già stata definita.
        // Se però identifico un nuovo picco che ha un valore maggiore del precedente allora considero quello come
        // picco principale e imposto nuovamente la soglia inferiore
        if (last > slowWalk_H_Th && last > secondToLast && last > current)
        {
            if (last > lastPeekValue)
            {
                Serial.print("New Peek detected:\t");
                Serial.println(last);
                currentStep.peek = totalValueRead;
                lastPeekValue = last;
                if (last >= run_H_Th)
                {
                    lowerThreshold = run_L_Th;
                    currentStep.stepType = MotionState::run;
                }
                else if (last >= walk_H_Th)
                {
                    lowerThreshold = walk_L_Th;
                    currentStep.stepType = MotionState::walk;
                }
                else if (last >= slowWalk_H_Th)
                {
                    lowerThreshold = slowWalk_L_Th;
                    currentStep.stepType = MotionState::slowWalk;
                }
            }
        }

        if (secondToLast < last && last < current && current < lowerThreshold)
        {

            currentStepState = StepWindowState::NoOperation;
            currentStep.end = totalValueRead - 2;
            Serial.print("step Ended\t");
            Serial.print("size: ");
            Serial.print(currentStep.end - currentStep.start);
            Serial.print("\t");
            Serial.print("type: ");
            Serial.println(currentStep.stepType);

            // Effettuo il push dello step riconosciuto dentro l'array e resetto
            // il passo corrente
            mosteRecentStep = currentStep;
            currentStep = {0, 0, 0, MotionState::unknow};
            lowerThreshold = 0;
            return true;
        }
        break;
    }

    return false;
}
