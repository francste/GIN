# Freno pneumatico — Arduino Nano Every

Primo controllo sperimentale per un cilindro frenato da una camera d'aria:
la pompa aumenta la frenatura, mentre uno sfiato controllato riduce lentamente
la pressione. Lo sketch cerca una **media di un fronte encoder ogni 600 ms**,
includendo il tempo in cui il cilindro resta fermo.

Si contano **salita e discesa**, come richiesto: `CHANGE` su **D14, cioè A0**.
Un impulso completo alto/basso genera due conteggi; il target equivale quindi
a un impulso completo ogni 1200 ms. Il filtro temporale dei fronti è disattivato
di default, per contare anche gli impulsi stretti.

## Struttura e uso locale

```text
GIN/
├── README.md
├── sketchbook/
│   └── FrenoPneumatico/
│       ├── FrenoPneumatico.ino
│       └── Controller.h
└── tests/
    ├── controller_test.cpp
    └── mock/
        ├── Arduino.h
        └── util/
            └── atomic.h
```

Il repository locale può essere la cartella `GIN`. In Arduino IDE impostare
la **Posizione sketchbook** nelle preferenze alla cartella `GIN/sketchbook`
del proprio computer, oppure aprire direttamente
`sketchbook/FrenoPneumatico/FrenoPneumatico.ino`.
Arduino richiede che la cartella dello sketch e il file `.ino` principale
abbiano lo stesso nome: qui entrambi si chiamano `FrenoPneumatico`.

Tutti i sorgenti del firmware sono nella stessa cartella dello sketch.
`FrenoPneumatico.ino` contiene pin, encoder, seriale e funzioni `setup`/`loop`;
`Controller.h` contiene la regolazione e i suoi parametri. Mantenere due file
permette di leggere e provare la regolazione separatamente dall'hardware.
È possibile riunirli in un `.ino`, ma non è necessario per compilare con
Arduino IDE. Non servono librerie locali aggiuntive in `sketchbook/libraries`.

`tests/` è riservata alla simulazione sul PC e non va copiata nella cartella
dello sketch. I suoi file `Arduino.h` e `util/atomic.h` sostituiscono le API
hardware soltanto quando i test vengono compilati con `-I tests/mock`.

### A cosa serve `util/atomic.h`

Nel firmware, `<util/atomic.h>` è fornito dalla toolchain AVR del pacchetto
Arduino megaAVR Boards. L'interrupt dell'encoder aggiorna variabili a 32 bit;
su questo microcontrollore a 8 bit, leggerle richiede più istruzioni.
`volatile` da solo non impedisce che un interrupt intervenga durante la lettura.
In `readEncoder()`, `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)` sospende gli interrupt
per copiare insieme conteggio, ultimo istante e periodo medio, poi ripristina
lo stato precedente degli interrupt.

Il file `tests/mock/util/atomic.h` contiene una versione per i test: il suo
`for` esegue il blocco una sola volta e non disabilita interrupt. Questo è
adeguato alla simulazione sequenziale, ma non verifica la concorrenza reale
tra interrupt e `loop()`. Non usare questo file sul Nano Every: la compilazione
Arduino deve trovare la versione AVR originale.

Dopo aver installato Arduino megaAVR Boards, dalla radice del repository
si può compilare anche con Arduino CLI:

```sh
arduino-cli compile --fqbn arduino:megaavr:nona4809:mode=off sketchbook/FrenoPneumatico
```

## Strategia

Una regolazione che reagisce ripetutamente a ogni fronte durante il ritardo
pneumatico può caricare troppa pressione. Questo controllore:

1. Riconosce un arresto **presunto** dopo `quietMs` senza fronti.
2. Conta i fronti dello scatto successivo, anche mentre la pompa è accesa
   e mentre si attende il suo effetto.
3. Dal terzo fronte stima la velocità usando la media degli ultimi **due**
   intervalli. In questo modo media la durata del livello alto e quella del
   livello basso, che possono essere diverse.
4. Se questa media è inferiore a 600 ms, calcola un impulso proporzionale
   all'errore, corretto lentamente dai risultati dei cicli precedenti.
5. Scarta richieste inferiori a 50 ms. Dopo lo spegnimento attende 800 ms
   prima di considerare un altro impulso; durante l'attesa non accoda richieste.
6. Alla chiusura del ciclo corregge lentamente la durata. Se manca un arresto,
   aggiorna ogni 6 secondi su campioni disgiunti. Questo evita di restare senza
   feedback durante una rotazione continua.

La richiesta è:

```text
errore_rapido = (600 - intervallo_medio_ms) / 600
richiesta_ms = limita(bias_ms + 100 * errore_rapido, 0, 200)
```

Si aziona la pompa soltanto con velocità sopra soglia e richiesta almeno 50 ms.
Il `bias` iniziale è 25 ms; può diventare negativo, così il controllo può
saltare impulsi quando gli scatti sono rapidi ma la media complessiva è bassa.
Il primo fronte dopo una pausa viene contato: da solo non permette di misurare
la velocità e non comanda la pompa. Con questa prima versione servono almeno
tre fronti nello scatto per attivare la correzione rapida.

Per un ciclo di durata `T` ms con `N` fronti, il conteggio desiderato è:

```text
N_target = T / 600
errore_medio = 600 * N / T - 1
```

Per esempio, 4 fronti in 2400 ms rispettano il target, anche se arrivano tutti
in uno scatto. Un numero fisso di fronti per scatto sarebbe un setpoint diverso:
non garantirebbe un fronte ogni 600 ms quando varia la durata della pausa.

L'errore medio viene filtrato con una media esponenziale (`alpha = 0,25`).
Fuori dalla zona morta del ±10%, il bias cambia di `5 * errore_filtrato` ms,
con una variazione massima di ±5 ms per aggiornamento. Il bias è limitato
tra −100 e 200 ms. I campioni usati nel recupero a 6 secondi non vengono
contati di nuovo alla chiusura del ciclo.

Il primo ciclo parte dal comando `a`; quelli successivi vanno da un arresto
presunto al successivo, includendo la pausa. L'arresto è una stima dalla
mancanza di fronti: uno spostamento più piccolo della risoluzione encoder
rimane invisibile.

## Hardware e caricamento

| Segnale | Pin | Livello/modalità |
| --- | --- | --- |
| Preimpostazione | D11 | HIGH |
| Preimpostazione | D6 | HIGH |
| Preimpostazione | D4 | LOW |
| Comando pompa | D3 | HIGH acceso, LOW spento |
| Encoder ottico | D14 / A0 | Interrupt CHANGE, INPUT_PULLUP |

I livelli vengono precaricati prima di impostare i pin come uscite. Al boot
D3 resta LOW e il controllo è disarmato. D3 comanda lo stadio di potenza
del motore, con protezione per il carico induttivo; il motore DC non va
alimentato direttamente dal GPIO. Encoder e scheda devono avere massa
comune e livelli compatibili con il Nano Every. Se l'encoder ha già un
pilotaggio adeguato, impostare `ENCODER_PULLUP = false`.

Aprire `sketchbook/FrenoPneumatico/FrenoPneumatico.ino` nell'IDE Arduino,
mantenendo `Controller.h` nella stessa cartella. Installare **Arduino megaAVR
Boards**, scegliere **Arduino Nano Every**, e scegliere **Registers emulation:
None (ATMEGA4809)**. Non selezionare il Nano classico ATmega328P.

## Primo test sul prototipo

1. Aprire il monitor seriale a **115200 baud**, con terminazione **Newline**.
   Con pompa disarmata, muovere il cilindro e verificare entrambi i fronti.
   La colonna `fronti_totali` del log `STATUS` mostra il contatore anche
   quando il controllo è disarmato.
2. Con condizioni meccaniche ripetibili, inviare `p 50`, eventualmente poi
   `p 100`. Si tratta di impulsi manuali singoli, utili per osservare il ritardo
   e la capacità effettiva della pompa. Rispettare l'attesa dopo ogni impulso.
   Durante questa misura il controllo automatico resta disarmato.
3. Stimare il tempo tra fine impulso ed effetto frenante. Impostare per esempio
   `d 1200` se 800 ms non bastano. Misurare anche il silenzio encoder:
   `q` deve superare i normali intervalli tra fronti del moto regolare e
   permettere di distinguere le soste reali. Il valore iniziale è `q 1800`.
   Se le soste sono più brevi, i cicli vengono uniti e interviene il recupero
   a finestre; `quietMs` va scelto sul comportamento osservato.
4. Iniziare con cilindro fermo e inviare `a`. Eseguire una prova breve,
   per esempio 30–60 secondi, con coppia applicata e sfiato ripetibili.
   Inviare `s` per spegnere D3 e disarmare; lo sfiato resta passivo.
5. Conservare il log. Confrontare **somma dei fronti / somma dei tempi**
   di più cicli, evitando la media non pesata delle velocità di cicli
   con durate diverse. Modificare un parametro alla volta e ripetere.

I comandi di modifica funzionano a controllo disarmato e pompa spenta:

| Comando | Effetto |
| --- | --- |
| `a` | Avvia un test; azzera i conteggi del test e il budget, conserva il bias appreso |
| `s` | Spegne subito il comando pompa e disarma |
| `?` | Stampa i comandi e i formati dei log |
| `t 600` | Target in ms per fronte; intervallo ammesso 50–5000 ms |
| `d 800` | Attesa dopo fine impulso; intervallo ammesso 50–10000 ms |
| `q 1800` | Silenzio per arresto presunto; deve superare `t`, massimo 30000 ms |
| `b 25` | Imposta il bias iniziale; intervallo ammesso −100…200 ms |
| `p 50` | Impulso manuale; intervallo ammesso 50–200 ms |

I parametri modificati e il bias appreso risiedono in RAM. Un reset della
scheda ripristina i valori del sorgente. Per confrontare prove indipendenti
usare `s`, `b 25`, poi `a`.

Se la media è troppo alta, il bias tende a salire; se è troppo bassa,
tende a scendere. Se si osservano lunghi bloccaggi, verificare prima che
l'attesa sia sufficiente a evitare impulsi aggiuntivi prima dell'effetto
del precedente. Se l'impulso di 50 ms produce già una frenatura eccessiva,
il software può saltare impulsi ma non ridurli ulteriormente: servirà
intervenire su portata pompa o sfiato.

Per il primo test sono impostati limiti di **200 ms per impulso**,
**4 impulsi per scatto** e **3000 ms totali di pompa per test**.
Un superamento disarma il controllo: `FAULT,1` indica il limite di impulsi,
`FAULT,2` il budget totale. Riavviare con `a` soltanto dopo aver valutato
il risultato; aumentare i limiti in `BrakeConfig` dopo la caratterizzazione.
Questi limiti non misurano né garantiscono una pressione massima: il sistema
non dispone di un sensore di pressione o di uno scarico comandato.

## Interpretazione dei log

```text
PUMP,t_ms,durata_ms,richiesta_ms,bias_ms
OFF,t_ms
CYCLE,t_ms,fronti,ciclo_ms,fronti_target,errore_filtrato,bias_ms,min_intervallo_us
RATE,t_ms,fronti_campione,campione_ms,errore_filtrato,bias_ms
STATUS,t_ms,armato,pompa,fronti_scatto,pompa_totale_ms,bias_ms,fault,fronti_totali,last_fronte_ms
```

`CYCLE.fronti` include tutti i fronti validi dello scatto, senza azzeramenti
durante pompa e attesa. `min_intervallo_us` è il minimo della stima mediata
sugli ultimi due intervalli. L'errore filtrato può includere campioni `RATE`
precedenti, quindi non coincide sempre con l'errore grezzo dell'ultimo ciclo.
`fronti_totali` è il contatore cumulativo dalla partenza della scheda,
anche durante impulsi manuali; `last_fronte_ms` permette di osservare la
coda di movimento dopo `OFF`.

I log usano una coda non bloccante: un terminale lento non prolunga l'impulso
aspettando spazio nella UART. `TX_DROPPED,n` segnala perdita di byte di log;
in quel caso ripetere la misura senza inviare comandi seriali in continuazione.
La temporizzazione resta servita nel `loop()`, con risoluzione millisecondo:
verificare sul banco la durata effettiva di D3 e il ritardo meccanico.

## Verifiche eseguite nel cloud

Compilazione reale per `arduino:megaavr:nona4809:mode=off`, con CLI 1.4.1,
core megaAVR 1.8.8 e API 1.3.1. Il cloud usa AVR GCC Debian 14.2.0,
avr-libc 2.2.1 e binutils 2.43.50: il compilatore differisce da quello
distribuito col pacchetto Arduino standard. Sorgenti fissati a commit e
artefatti verificati tramite SHA256/indice Debian firmato.

Nel cloud i comandi sono:

```sh
/workspace/.tools/arduino/setup-nano-every.sh
/workspace/.tools/arduino/compile-nano-every.sh
```

La CLI segnala gli indici remoti e i tool di discovery non disponibili
in questo ambiente. Il core locale e il compilatore sono sufficienti per
compilare e generare l'HEX del Nano Every. Non è stato verificato il caricamento
USB: non c'è una scheda collegata al cloud.

I test nativi verificano sette gruppi: pin/entrambi i fronti, soglia minima e
encoder asimmetrico, conteggio durante ritardo/adattamento, media con soste,
rotazione continua/limiti, stop/manuale/rollover, seriale congestionata.

```sh
set -e
mkdir -p /tmp/gin-tests
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  -fsanitize=address,undefined -I tests/mock tests/controller_test.cpp \
  -o /tmp/gin-tests/controller_test
/tmp/gin-tests/controller_test
```

Se il runtime cloud segnala che LeakSanitizer non può funzionare sotto
`ptrace`, eseguire il binario con `ASAN_OPTIONS=detect_leaks=0`.
AddressSanitizer e UndefinedBehaviorSanitizer rimangono attivi; è disabilitata
soltanto la ricerca delle perdite di memoria.

Queste verifiche riguardano firmware e temporizzazione simulata. Il ritardo
pneumatico, l'attrito e la convergenza reale della velocità restano da misurare
sul prototipo con la procedura sopra.
