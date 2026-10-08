#include <Arduino.h>
#include <util/atomic.h>
#include <stdio.h>

// Parametri della prima prova.
constexpr uint8_t MOTOR_PIN = 3;
constexpr uint8_t ENCODER_PIN = A0;             // D14, senza pull-up interno.
constexpr uint8_t DEBUG_PIN = 12;               // PE1 sul Nano Every.
constexpr uint32_t ENCODER_HOLDOFF_US = 2000;
constexpr uint32_t PRE_GONFIAGGIO_MS = 1500;
constexpr uint32_t TIMEOUT_ARRESTO_MS = 300;
constexpr uint32_t MS_PER_FRONTE = 600;
constexpr uint16_t IMPULSO_BLOCCAGGIO_MS = 150; // Prima accensione di ogni sequenza.
constexpr uint16_t IMPULSO_MANTENIMENTO_MS = 70;
constexpr uint16_t RICHIESTA_INIZIALE_MS = 200;
constexpr uint16_t PASSO_MS = 20;
constexpr uint16_t RICHIESTA_MINIMA_MS = 50;
constexpr uint16_t RICHIESTA_MASSIMA_MS = 400;
static_assert(IMPULSO_BLOCCAGGIO_MS > 0 && IMPULSO_BLOCCAGGIO_MS <= RICHIESTA_MASSIMA_MS &&
              IMPULSO_MANTENIMENTO_MS > 0 && IMPULSO_MANTENIMENTO_MS <= RICHIESTA_MASSIMA_MS,
              "Le durate degli impulsi devono essere tra 1 e RICHIESTA_MASSIMA_MS");

enum Stato { PRE_GONFIAGGIO, ATTENDI_ARRESTO, ATTENDI_FRONTE,
             MOTOR_ON, MANTENIMENTO, FERMO };
Stato stato = FERMO;
bool ingressoStato = true;                     // true soltanto dopo una transizione.
uint32_t inizioStatoMs = 0;
uint16_t tempoRichiestoMs = RICHIESTA_INIZIALE_MS;
uint32_t inizioSequenzaMs = 0;
uint16_t numeroImpulsi = 1;
uint16_t indiceImpulso = 0;                    // 0 = prima accensione della sequenza.
bool mantenimentoAcceso = false;

// Campioni presi a ogni inizio sequenza: le accensioni intermedie non li cambiano.
volatile uint32_t encoderTotale = 0;
uint32_t ultimoFronteValidoUs = 0;              // Usato soltanto nell'ISR.
bool fronteValidoRicevuto = false;             // Accetta anche il primo fronte a t=0.
// L'ISR accende subito e registra i confini esatti del nuovo bloccaggio.
volatile bool encoderPronto = false;
volatile bool bloccaggioRichiesto = false;
volatile uint32_t bloccaggioMs = 0;
volatile uint32_t bloccaggioFronti = 0;
bool precedenteSequenzaValida = false;
uint32_t precedenteSequenzaMs = 0;
uint32_t totaleAllaSequenza = 0;
uint32_t ultimoPeriodoMs = 0;
uint32_t frontiPeriodo = 0;

void encoderISR() {
  // Copia il livello grezzo: sul debug si vedono anche i rimbalzi scartati.
  digitalWrite(DEBUG_PIN, digitalRead(ENCODER_PIN));
  const uint32_t nowUs = micros();
  if (fronteValidoRicevuto &&
      uint32_t(nowUs - ultimoFronteValidoUs) < ENCODER_HOLDOFF_US) return;
  // Il holdoff parte dal fronte accettato; i rimbalzi non lo prolungano.
  ultimoFronteValidoUs = nowUs;
  fronteValidoRicevuto = true;
  ++encoderTotale;                             // CHANGE: conta salita e discesa.
  if (encoderPronto) {
    digitalWrite(MOTOR_PIN, HIGH);             // Prima di calcoli, log e prossimo loop.
    bloccaggioMs = millis();
    bloccaggioFronti = encoderTotale;
    encoderPronto = false;                    // Altri fronti si contano, senza riavviare il blocco.
    bloccaggioRichiesto = true;
  }
}

void attendiEncoder() {
  // Non spegne un bloccaggio appena richiesto dall'ISR.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (!bloccaggioRichiesto) {
      digitalWrite(MOTOR_PIN, LOW);
      encoderPronto = true;
    }
  }
}

uint16_t durataImpulsoMs() {
  return indiceImpulso == 0 ? IMPULSO_BLOCCAGGIO_MS : IMPULSO_MANTENIMENTO_MS;
}

void stampaImpulso(int16_t correzioneMs) {
  char riga[64];
  int lunghezza;
  if (indiceImpulso == 0) {
    // Solo il bloccaggio ha il campione completo e la correzione della richiesta.
    lunghezza = snprintf(riga, sizeof(riga),
                         "Imp:%3u Corr:%+3d Dt:%10lu Fr:%10lu Req:%3u N:%3u/%3u\n",
                         unsigned(durataImpulsoMs()), int(correzioneMs),
                         (unsigned long)ultimoPeriodoMs,
                         (unsigned long)frontiPeriodo, unsigned(tempoRichiestoMs),
                         unsigned(indiceImpulso + 1), unsigned(numeroImpulsi));
  } else {
    // Mantenimento: stampa soltanto il progressivo nella sequenza.
    lunghezza = snprintf(riga, sizeof(riga), "%u/%u\n",
                         unsigned(indiceImpulso + 1), unsigned(numeroImpulsi));
  }
  // Nessuna attesa per la UART: stampa solo se entra l'intera riga.
  if (lunghezza > 0 && lunghezza < int(sizeof(riga)) &&
      Serial.availableForWrite() >= lunghezza) Serial.print(riga);
}

void cambiaStato(Stato nuovoStato) {
  if (nuovoStato == stato) return;
  if (nuovoStato == FERMO || nuovoStato == PRE_GONFIAGGIO) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      encoderPronto = false;
      bloccaggioRichiesto = false;
    }
  }
  stato = nuovoStato;
  ingressoStato = true;
}

void preparaSequenza(uint32_t now, uint32_t totale) {
  if (precedenteSequenzaValida) {
    ultimoPeriodoMs = uint32_t(now - precedenteSequenzaMs);
    frontiPeriodo = uint32_t(totale - totaleAllaSequenza);
    const uint64_t tempoObiettivoMs = uint64_t(frontiPeriodo) * MS_PER_FRONTE;

    // Lento: meno gonfiaggio. Rapido: piu' gonfiaggio. Uguale: nessuna modifica.
    int32_t nuovaRichiestaMs = tempoRichiestoMs;
    if (ultimoPeriodoMs > tempoObiettivoMs) nuovaRichiestaMs -= PASSO_MS;
    else if (ultimoPeriodoMs < tempoObiettivoMs) nuovaRichiestaMs += PASSO_MS;
    if (nuovaRichiestaMs < RICHIESTA_MINIMA_MS) nuovaRichiestaMs = RICHIESTA_MINIMA_MS;
    if (nuovaRichiestaMs > RICHIESTA_MASSIMA_MS) nuovaRichiestaMs = RICHIESTA_MASSIMA_MS;
    tempoRichiestoMs = uint16_t(nuovaRichiestaMs);
  }
  precedenteSequenzaMs = inizioSequenzaMs = now;
  totaleAllaSequenza = totale;
  precedenteSequenzaValida = true;

  // Il bloccaggio e' sempre presente. Il mantenimento usa solo il residuo.
  numeroImpulsi = 1;
  if (ultimoPeriodoMs > 0 &&
      tempoRichiestoMs > uint32_t(IMPULSO_BLOCCAGGIO_MS) + IMPULSO_MANTENIMENTO_MS) {
    numeroImpulsi += (tempoRichiestoMs - IMPULSO_BLOCCAGGIO_MS) / IMPULSO_MANTENIMENTO_MS;
  }
  // Prima sequenza: manca Dt, quindi resta il solo impulso di bloccaggio.
  if (numeroImpulsi > 1) {
    // La distanza fra avvii deve contenere anche l'impulso piu' lungo e 1 ms spento.
    const uint16_t impulsoPiuLungoMs = IMPULSO_BLOCCAGGIO_MS > IMPULSO_MANTENIMENTO_MS
                                       ? IMPULSO_BLOCCAGGIO_MS : IMPULSO_MANTENIMENTO_MS;
    const uint32_t capienza = ultimoPeriodoMs / (uint32_t(impulsoPiuLungoMs) + 1);
    if (numeroImpulsi > capienza) numeroImpulsi = capienza > 0 ? uint16_t(capienza) : 1;
  }
}

void aggiornaFreno(uint32_t now) {
  bool nuovoBloccaggio = false;
  uint32_t avvioBloccaggioMs = 0, frontiAlBloccaggio = 0;
  // Copia atomica della richiesta e dei campioni a 32 bit scritti nell'ISR.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (bloccaggioRichiesto) {
      avvioBloccaggioMs = bloccaggioMs;
      frontiAlBloccaggio = bloccaggioFronti;
      bloccaggioRichiesto = false;
      nuovoBloccaggio = true;
    }
  }
  if (nuovoBloccaggio) {
    now = millis();                          // Il fronte puo' essere arrivato dopo il now del loop.
    indiceImpulso = 0;                        // Annulla la vecchia sequenza di mantenimento.
    cambiaStato(MOTOR_ON);
  }

  // Una transizione esegue subito il once del nuovo stato, nello stesso loop.
  for (;;) {
    const bool once = ingressoStato;
    ingressoStato = false;

    switch (stato) {
      case PRE_GONFIAGGIO:
        if (once) {                            // ONCE: prepara una nuova prova.
          tempoRichiestoMs = RICHIESTA_INIZIALE_MS;
          precedenteSequenzaValida = false;
          ultimoPeriodoMs = frontiPeriodo = 0;
          indiceImpulso = 0;
          numeroImpulsi = 1;
          mantenimentoAcceso = false;
          inizioSequenzaMs = 0;
          inizioStatoMs = now;
          digitalWrite(MOTOR_PIN, HIGH);
        }
        // ALWAYS: termina il pregonfiaggio dopo 1,5 secondi.
        if (uint32_t(now - inizioStatoMs) >= PRE_GONFIAGGIO_MS) {
          cambiaStato(ATTENDI_ARRESTO);
          continue;
        }
        break;

      case ATTENDI_ARRESTO:
        if (once) {                            // ONCE: spegni e avvia il timeout.
          digitalWrite(MOTOR_PIN, LOW);
          inizioStatoMs = now;
        }
        // ALWAYS: attesa fissa di 300 ms, indipendente dai fronti.
        if (uint32_t(now - inizioStatoMs) >= TIMEOUT_ARRESTO_MS) {
          cambiaStato(ATTENDI_FRONTE);
          continue;
        }
        break;

      case ATTENDI_FRONTE:
        if (once) attendiEncoder();            // ONCE: abilita il primo bloccaggio dall'ISR.
        // ALWAYS: resta in attesa; la richiesta ISR e' gestita all'inizio del loop.
        break;

      case MOTOR_ON:
        if (once) {                            // ONCE: l'ISR ha gia' acceso il motore.
          mantenimentoAcceso = false;
          const uint16_t richiestaPrecedenteMs = tempoRichiestoMs;
          preparaSequenza(avvioBloccaggioMs, frontiAlBloccaggio);
          const int16_t correzioneMs = int16_t(tempoRichiestoMs) - int16_t(richiestaPrecedenteMs);
          inizioStatoMs = avvioBloccaggioMs;    // 150 ms dall'accensione effettiva nell'ISR.
          stampaImpulso(correzioneMs);
        }
        // ALWAYS: i fronti si contano, senza allungare il bloccaggio.
        if (uint32_t(now - inizioStatoMs) >= IMPULSO_BLOCCAGGIO_MS) {
          cambiaStato(MANTENIMENTO);
          continue;
        }
        break;

      case MANTENIMENTO:
        if (once) {                            // ONCE: spegni e abilita il blocco su nuovo fronte.
          indiceImpulso = 1;
          mantenimentoAcceso = false;
          inizioStatoMs = now;
          attendiEncoder();
        }
        // ALWAYS: gestisci i 70 ms e gli avvii programmati; alla fine resta in attesa.
        if (mantenimentoAcceso) {
          if (uint32_t(now - inizioStatoMs) >= IMPULSO_MANTENIMENTO_MS) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
              // Una richiesta arrivata durante questo loop deve lasciare il motore acceso.
              if (!bloccaggioRichiesto) digitalWrite(MOTOR_PIN, LOW);
            }
            mantenimentoAcceso = false;
            ++indiceImpulso;
            inizioStatoMs = now;
          }
        } else if (indiceImpulso < numeroImpulsi &&
                   uint32_t(now - inizioSequenzaMs) >=
                     uint64_t(indiceImpulso) * ultimoPeriodoMs / numeroImpulsi &&
                   uint32_t(now - inizioStatoMs) > 0) {
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (!bloccaggioRichiesto) {
              digitalWrite(MOTOR_PIN, HIGH);
              mantenimentoAcceso = true;
              inizioStatoMs = now;
            }
          }
          if (mantenimentoAcceso) stampaImpulso(0);
        }
        break;

      case FERMO:
        if (once) {                            // ONCE: stop manuale della pompa.
          digitalWrite(MOTOR_PIN, LOW);
        }
        // ALWAYS: resta fermo fino al comando a.
        break;
    }
    return;
  }
}

void leggiComandi() {
  for (uint8_t i = 0; i < 8 && Serial.available() > 0; ++i) {
    const char comando = char(Serial.read());
    if (comando == 's') cambiaStato(FERMO);
    else if (comando == 'a' && stato == FERMO) cambiaStato(PRE_GONFIAGGIO);
  }
}

void setup() {
  // Ingressi fisici fuori dalla mappatura Arduino: buffer attivo, senza pull-up o interrupt.
  PORTD.DIRCLR = PIN6_bm; PORTD.PIN6CTRL = 0;  // PD6.
  PORTA.DIRCLR = PIN6_bm; PORTA.PIN6CTRL = 0;  // PA6.
  // Prima la direzione, poi il livello: il core megaAVR richiede questo ordine.
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(ENCODER_PIN, INPUT);
  digitalWrite(ENCODER_PIN, LOW);
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, LOW);
  digitalWrite(DEBUG_PIN, digitalRead(ENCODER_PIN));
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  Serial.begin(115200);
  Serial.println("Avvio freno");
  cambiaStato(PRE_GONFIAGGIO);
  aggiornaFreno(millis());                    // Esegue subito il once iniziale.
}

void loop() {
  leggiComandi();                             // Lo stop viene eseguito in questo loop.
  aggiornaFreno(millis());
}
