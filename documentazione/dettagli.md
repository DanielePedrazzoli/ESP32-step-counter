## Campionamento
In generale l'accelerazione è un valore che cambia rapidamente (considerando che si devono anche leggere i picchi) di conseguenza è opportuno usare una frequenza alta nel campionare. Detto questo però penso che la massima frequenza alla quale campionare possa essere circa 400 Hz.

## FIFO
Considerando che è necessario prendere i dati dall'acceleromentro abbiamo che
- 16 bit = 2 byte per ogni asse dell'accelerometro
- 16 bit = 2 byte per ogni asse del giroscopio 

questo quindi ci porta a

$$
  n_{byte} = (2 \text{ byte} * 3 \text{ assi}) + (2 \text{ byte} * 3 \text{ assi}) = 12 \text{ byte}
$$

Considerando che la FIFO ha una capacità massima di 1024 byte questo porterebbe ad un massimo di:

$$
    1024 / 12 \approx 85 \text{ campionamenti}
$$

Cosndierando la frequenza di clock dell'I2C di 400 kHz possiamo calcolare che la comunicazione dovrebbe durare approssimativamente

$$
    t_{com} = \frac{85 * 8 }{400 * 10^3} = 1.7 * 10^{-3} = 1.7 ms
$$

Mentre i dati verrebbero campionati ogni

$$
    t_{camp} = \frac{1}{400} = 25 ms 
$$

## Filtro
Il sensore fornisce i dati del giroscopio già filtratri con un filtro passa alto digitale integrato mentre i dati del'acceleromento sono _raw_.
Una volta acquisiti i dati l'ESP li dovrà filtrare per rimuovere il rumore o piccole variazioni non significative.
Per fare quetso ho pensato ad un filtraggio digitale FIR passa basso con una frequenza di taglio a 10 Hz poichè la variazione di accelerazione dovuta ad un passo difficilemnte supera questo valore.
from 0 Hz to 10 Hz , 1, 1db
from 20 Hz to  200Hz, 0, -40db
campFreq = 400Hz


## Analisi passi
Sono ancora incerto sull'algoritmo di riconoscimento passi. La cosa più intuitiva potrebbe essere un riconoscimento di picco. Quindi fissata una certa soglia (treshold) ogni valore al di sopra di essa verrà considerato un passo. 
Ovviamente questo è un medoto molto impreciso poiché un semplice spostamento del sensore potrerebbe ad una falsa lettura del passo.
Inoltre è necessario considerare anche il fatto che il sensore non sempre posizionato parallelamente al terreno (di conseguenza asse x e asse y hanno valori quasi nulli mentre la gravità è segnata in pieno dall'asse z)

In [questo articolo](https://arxiv.org/pdf/1801.02336#:~:text=The%20algorithm%20keeps%20track%20of,end%20and%20the%20start%20points.) si affronta anche questo problema.
Per risolvere il problema dell'orietamento del sensore calcola la magnitudine come

$$
    mag[t] = \sqrt{x[t]^2 + y[t]^2 + z[t]^2} 
$$

Dove i valori $x,y,z$ sono i valori dell'accelerometro nell'asse corrispondente all'istante di tempo $t$
Questo effettivamente permette di trasoformare un'analisi tri-dimensionale in una mono-dimensionale.
Inoltre per escludere la forza di gravità dall'analisi viene calcolata la magnitudine netta come

$$
    netmag[t] = mag[t] - avgmag
$$

Succesisvamente viene applicato un filtro passa-basso per rimuovere il rumore e un filtro.


Per individuare i passi viene usato questo algoritmo che si divide in 2 parti:

##### Parte 1 (riconoscimento picco)
- mantenere una finestra di valori presi da (netmag)
- controllare se i valori superano la soglia impostat 
- individuare il picco tra questi (il valore maggiore) e salvare la finestra di valori in un array

##### Parte 2 (riconoscimento passo)
- Di tutte le finestre salvate osservo se esiste una intersezione. In questo caso significa che è stato rilevato un picco (e quindi un passo) in un lasso di tempo estremamente stretto quindi probabilmente è un falso picco.

Contare il numero di passi è a questo punto molto semplice, infatti è sufficente contare il numero di finestre rilevate che non si intersecano tra di loro. Nel caso 2 o più fiestre di intersechino tra di loro allora viene contato soltatno come 1 passo

---

## Motion sensing
Il sesning del movimento viene effettuato tramite i passi ottenuti.
Ogni volta che viene rilevato un passo viene preso il tempo tramite la millis() e calcolata la media di tempo assieme agli ultimi passi (per iniziare si tiene una storia di 5 passi).

$$
    avgStepPeriod = \frac{\sum_{i=0}^5 stepsTimes[i]}{5}
$$

Poi, calcolando l'inverso di questo valore si può ottenere la frequenza dei passi

$$
    freqStep = \frac{1}{avgStepPeriod}
$$

Questa frequenza verrà poi passata in un contorllo a soglia per determinare lo stato attuale.

