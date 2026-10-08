# Freno pneumatico — prova semplice su Arduino Nano Every

Il firmware precarica la camera d'aria e, al primo fronte encoder di ogni
scatto, accende la pompa per un solo impulso. Dopo il blocco presunto attende
il prossimo fronte e puo' ridurre l'impulso di 20 ms.

## Sequenza

1. All'avvio accende la pompa per **2500 ms**, poi la spegne.
   I fronti arrivati durante la precarica vengono ignorati.
2. Il primo fronte dopo la precarica avvia subito un impulso di **200 ms**.
3. Conta tutti i fronti dello scatto, compreso il primo e quelli arrivati
   durante l'impulso. Al termine dell'impulso la pompa si spegne.
4. Attende **300 ms senza fronti** per considerare il freno bloccato.
   Ogni nuovo fronte prima del timeout incrementa `n` e riavvia il timeout;
   non accende nuovamente la pompa e non prolunga l'impulso.
5. Al primo fronte del nuovo scatto misura `T`: il tempo dal primo fronte
   dello scatto precedente al primo del nuovo, includendo moto e pausa.
   Se **T > n × 600 ms**, riduce l'impulso di **20 ms**, fino a un minimo
   di **20 ms**. Con uguaglianza o tempo inferiore mantiene la durata.
6. Il nuovo fronte e' il primo del nuovo conteggio; avvia un solo impulso
   con la durata appena calcolata e ripete la sequenza.

Esempio: se il primo scatto produce 4 fronti, il confronto e' con 2400 ms.
Un nuovo primo fronte dopo 2401 ms porta l'impulso da 200 a 180 ms; dopo
2400 ms o meno lo lascia a 200 ms. La durata puo' solo diminuire: questo
algoritmo non la aumenta quando il moto e' troppo rapido.

Il timeout conferma un **blocco presunto dall'encoder**: non misura la pressione
ne' uno spostamento piu' piccolo della risoluzione dell'encoder. Il valore di
300 ms e' una scelta iniziale da verificare sul banco. Con 1800 ms, uno scatto
con 1–3 fronti farebbe quasi sempre diminuire l'impulso per la sola attesa.
Se non si verifica mai il timeout, il firmware continua a contare e attende
il blocco a pompa spenta, senza aggiungere altri impulsi.

## Sorgente e parametri

Tutto il firmware e' in un unico file commentato:

```text
GIN/
├── README.md
├── sketchbook/
│   └── FrenoPneumatico/
│       └── FrenoPneumatico.ino
└── tests/
    ├── controller_test.cpp
    └── mock/
        ├── Arduino.h
        └── util/
            └── atomic.h
```

I parametri sono all'inizio del `.ino`:

| Parametro | Valore iniziale | Significato |
| --- | --- | --- |
| `PRECARICA_MS` | 2500 | Accensione pompa all'avvio |
| `IMPULSO_INIZIALE_MS` | 200 | Primo impulso di frenatura |
| `PASSO_MS` | 20 | Riduzione per ciclo troppo lento |
| `IMPULSO_MINIMO_MS` | 20 | Durata minima; impostare 0 per consentire pompa spenta |
| `MS_PER_FRONTE` | 600 | Tempo obiettivo per ciascun fronte |
| `TIMEOUT_BLOCCO_MS` | 300 | Assenza di fronti per blocco presunto |

Il codice ha quattro stati operativi: `PRECARICA`, `ATTENDI_FRONTE`,
`IMPULSO`, `ATTENDI_BLOCCO`, piu' lo stato manuale `FERMO`.
La temporizzazione usa `millis()`, senza `delay()`: l'encoder e il comando
seriale di stop vengono serviti anche durante le accensioni.

## Collegamenti e IDE Arduino

| Segnale | Pin | Configurazione |
| --- | --- | --- |
| Preimpostazione | D11 | HIGH |
| Preimpostazione | D6 | HIGH |
| Preimpostazione | D4 | LOW |
| Comando pompa | D3 | HIGH acceso, LOW spento |
| Encoder | D14 / A0 | INPUT, senza pull-up interno, interrupt CHANGE |

Si contano sia salita sia discesa: un impulso completo alto/basso vale due
fronti. L'encoder deve fornire livelli definiti e compatibili con il Nano Every,
con massa comune alla scheda. D3 comanda lo stadio di potenza del motore.

Il repository locale puo' essere la cartella `GIN`. Nelle preferenze dell'IDE
impostare **Posizione sketchbook** sulla cartella `GIN/sketchbook`, oppure
aprire direttamente `sketchbook/FrenoPneumatico/FrenoPneumatico.ino`.
La cartella dello sketch ha lo stesso nome del file `.ino`, come richiesto
da Arduino. Installare **Arduino megaAVR Boards**, scegliere **Arduino Nano
Every** e **Registers emulation: None (ATMEGA4809)**.

All'alimentazione o reset parte automaticamente la precarica di 2,5 secondi.
Non serve inviare un comando di avvio. Per una nuova prova dopo uno stop,
`a` ripete la precarica e ripristina l'impulso a 200 ms.

## Seriale

Monitor seriale a **115200 baud**. Sono rimasti due comandi:

| Comando | Effetto |
| --- | --- |
| `s` | Spegne la pompa e ferma il controllo, anche durante la precarica |
| `a` | Da `FERMO`, avvia una nuova prova con precarica e impulso iniziale |

Si possono inviare con o senza terminazione di riga. `a` durante una prova
attiva viene ignorato. I parametri si modificano nel sorgente.

Una riga di stato al secondo ha il formato:

```text
S,stato,P,impulso_ms,N,fronti_scatto,T,periodo_precedente_ms
```

Stati: 0=precarica, 1=attesa fronte, 2=impulso, 3=attesa blocco, 4=fermo.
`P` indica l'impulso di frenatura, anche durante la precarica da 2500 ms.
`N` include tutti i fronti dello scatto; resta visibile dopo il blocco.
`T` e' il tempo del ciclo appena confrontato con il suo conteggio precedente;
vale 0 finche' non inizia il secondo scatto.
La riga viene saltata se il buffer seriale non ha spazio sufficiente, per
non attendere la seriale mentre si devono rispettare i tempi della pompa.

## Perche' resta util/atomic.h

L'interrupt conta i fronti e registra il primo e l'ultimo istante. Su questo
microcontrollore a 8 bit la copia dei valori a 32 bit deve essere protetta
con `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)`: interrompe brevemente gli interrupt
per copiare e azzerare il conteggio pendente, poi ripristina lo stato precedente.
La versione reale di `<util/atomic.h>` e' fornita dalla toolchain AVR.

`tests/mock/util/atomic.h` e `tests/mock/Arduino.h` servono soltanto ai test
sequenziali sul PC; non devono essere copiati nello sketch. Il mock atomico
esegue il blocco una volta e non verifica la concorrenza reale degli interrupt.

## Verifiche

Dalla radice del repository, con il core installato:

```sh
arduino-cli compile --fqbn arduino:megaavr:nona4809:mode=off sketchbook/FrenoPneumatico
```

Nel cloud e' disponibile il comando:

```sh
/workspace/.tools/arduino/compile-nano-every.sh
```

La compilazione cloud usa Arduino CLI 1.4.1, megaAVR 1.8.8, API 1.3.1 e
AVR GCC Debian 14.2.0, diverso da quello del pacchetto Arduino standard.
Gli indici remoti e i tool di discovery non disponibili non impediscono
la compilazione con il core locale. Il caricamento USB non e' verificato.

I test verificano dieci gruppi: precarica/pin, impulso al primo fronte,
timeout rinnovato, soglia stretta e conteggio precedente, fronti accodati,
moto continuo, durata minima, stop/riavvio, rollover e seriale congestionata.

```sh
set -e
mkdir -p /tmp/gin-tests
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  -fsanitize=address,undefined -I tests/mock tests/controller_test.cpp \
  -o /tmp/gin-tests/controller_test
/tmp/gin-tests/controller_test
```

Se LeakSanitizer non puo' funzionare sotto `ptrace`, avviare il binario con
`ASAN_OPTIONS=detect_leaks=0`. Rimangono attivi AddressSanitizer e
UndefinedBehaviorSanitizer. I test simulati verificano il firmware; il tempo
per bloccare il freno e la risposta pneumatica vanno osservati sul prototipo.
