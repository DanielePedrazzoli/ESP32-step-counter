
## Sequenza di funzionamento completa
L'algoritmo è abbastanza semplice al momento. Ma sto progettando di ampliarlo.

#### Campionamento
- Ottenimento interrupt di lettura dati
- Controllo numero dati in FIFO
- Ottenimento del vettore di gravità dal DSP del sensore
- Eliminazione della gravità
- Calcolo del modulo per ottenere una analisi mono dimensionale
- Filtraggio in un filtro passa basso


#### Riconoscimento
- Se ho oltrepassato il threshold segno uno step e memorizzo tramite `millis()` il momento in cui è avvenuto
- Ignoro tutti gli altri valori fino a quando non sono al di sotto del threshold
- Se torno sopra il treshold prima di un certo lasso di tempo, allora ignoro il passo attuale
    Questo serve per rimuovere alcuni _falsi_ reilevamenti di passi dall'analisi

#### Analisi stato
In base a come vengono rilevati i passi è possibile determinare lo stato
- Ogni volta che viene aggiunto un passo aggiungerlo ad buffer circolare di qualche elemento
- Calcolare la media di questo buffer
    Questo rappresenta il tempo medio tra un inserimento e l'altro e quindi il tempo medio tra un passo e l'altro.
    L'inverso di questo valore rapopresenta la frequenza dei passi e quindi mi permette di ricavare uno stato di movimento tramite dei confronti usando delle soglie fisse.
    Tecnicamente questo valore sarebbe anche in grado di modificare la soglia di threshold[^notaSoglia].



---
## Sezioni specifiche

### Campionamento
In generale l'accelerazione è un valore che cambia rapidamente (considerando che si devono anche leggere i picchi) di conseguenza è opportuno usare una frequenza alta nel campionare. Detto questo però penso che la massima frequenza alla quale campionare possa essere circa 400 Hz.



### Filtro
Il sensore fornisce i dati del giroscopio già filtratri con un filtro passa alto digitale integrato mentre i dati del'acceleromento sono _raw_.
Una volta acquisiti i dati l'ESP li dovrà filtrare per rimuovere il rumore o piccole variazioni non significative.
Per fare quetso ho pensato ad un filtraggio digitale FIR passa basso con una frequenza di taglio a 10 Hz poichè la variazione di accelerazione dovuta ad un passo difficilemnte supera questo valore.
from 0 Hz to 10 Hz , 1, 1db
from 20 Hz to  200Hz, 0, -40db
campFreq = 400Hz

Osservando diversi articoli online ho visto che molti consigliano l'utilizzo di un filtro kalman. Dovrei aver trovato una libreria che mi permette di usarlo ma non ne sono molto certo nel del funzionamento ne della sua efficacia.
Inoltre non sapendo bene come funziona questo filtro risulterebbe difficile aggiustare correttamente i valori


#### Calibrazione
Se si mette una tensione di 3V3 sul pin 25 durante la fase di setup il dispositivo entra in automatico in auto calibrazione
L'alrogitmo di calibrazione effettua una serie ripetuta di misurazioni di ogni sensore e infine calcola la media dei valori per ogni sensore.
Tramite questa media calcola gli offset da applicare e, una volta applicati, li aggiusta in modo fine per fare in modo che gli errori rientrino entro determinati range.
Terminata questa sequenza i valori di offset vengono salvati nella memoria locale dell'ESP32 tramite la libreria [Preferences](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/preferences.html) che permette di salvare dati tramite una associazione "chiave-valore" nella flash del dispositivo.

La presenza di valori di offset precedenti viene controllata ad ogni setup
- Se vengono rilevati dei valori allora, a meno che il pin apposito non sia attivo, carica i valori presenti e prosegue con il programma
- Se non vengono rilevati allora parte in automatico la sequenza di calibrazione e poi il dispositivo verrà riavviato in automatico.

[^notaSoglia]: Una frequenza maggiore potrebbe significare che la soglia è troppo bassa e quindi servirebbe a regolarla.
Questo non dovrebbe essere però il metodo princiaple di regolazione della soglia dinamica