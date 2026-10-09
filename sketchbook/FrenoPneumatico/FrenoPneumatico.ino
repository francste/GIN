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
constexpr uint16_t RICHIESTA_MINIMA_MS = IMPULSO_BLOCCAGGIO_MS + IMPULSO_MANTENIMENTO_MS;
constexpr uint16_t RICHIESTA_INIZIALE_MS = RICHIESTA_MINIMA_MS; // 220 ms: blocco + mantenimento.
constexpr uint16_t KP_PER_MILLE = 1000;        // Kp = 1 ms di correzione per ms di errore medio.
constexpr uint16_t RICHIESTA_MASSIMA_MS = 2000;
static_assert(IMPULSO_BLOCCAGGIO_MS > 0 && IMPULSO_BLOCCAGGIO_MS <= RICHIESTA_MASSIMA_MS &&
              IMPULSO_MANTENIMENTO_MS > 0 &&
              uint32_t(IMPULSO_BLOCCAGGIO_MS) + IMPULSO_MANTENIMENTO_MS <= RICHIESTA_MASSIMA_MS,
              "La richiesta massima deve contenere bloccaggio e almeno un mantenimento");

enum Stato { PRE_GONFIAGGIO, ATTENDI_ARRESTO, ATTENDI_FRONTE,
             MOTOR_ON, MANTENIMENTO, FERMO };
Stato stato = FERMO;
bool ingressoStato = true;                     // true soltanto dopo una transizione.
uint32_t inizioStatoMs = 0;
uint16_t tempoRichiestoMs = RICHIESTA_INIZIALE_MS;
uint32_t inizioSequenzaMs = 0;
uint32_t periodoDistribuzioneMs = 0;
uint16_t numeroImpulsi = 2;
uint16_t indiceImpulso = 0;                    // 0 = prima accensione della sequenza.
bool mantenimentoAcceso = false;
uint32_t ultimoImpulsoMs = 0;                  // Accensione precedente, anche se il log e' saltato.

// Campioni presi a ogni inizio sequenza: le accensioni intermedie non li cambiano.
volatile uint32_t encoderTotale = 0;
uint32_t ultimoFronteValidoUs = 0;              // Usato soltanto nell'ISR.
bool fronteValidoRicevuto = false;             // Accetta anche il primo fronte a t=0.
// L'ISR segnala il fronte; soltanto il main loop comanda il motore.
volatile bool encoderPronto = false;
volatile bool bloccaggioRichiesto = false;
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
    encoderPronto = false;                    // Gli altri fronti si contano fino alla fine dei 150 ms.
    bloccaggioRichiesto = true;
  }
}

void attendiEncoder() {
  // Abilita la richiesta di bloccaggio su un nuovo fronte.
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
  char riga[80];
  int lunghezza;
  if (indiceImpulso == 0) {
    // Solo il bloccaggio ha il campione completo e la correzione della richiesta.
    lunghezza = snprintf(riga, sizeof(riga),
                         "Imp:%3u Corr:%+4d Dt:%10lu Fr:%5lu Req:%3u %u/%u t:0\n",
                         unsigned(durataImpulsoMs()), int(correzioneMs),
                         (unsigned long)ultimoPeriodoMs,
                         (unsigned long)frontiPeriodo, unsigned(tempoRichiestoMs),
                         unsigned(indiceImpulso + 1), unsigned(numeroImpulsi));
  } else {
    // t parte dal bloccaggio; d e' la distanza dall'accensione precedente.
    lunghezza = snprintf(riga, sizeof(riga), "%u/%u t:%10lu d:%10lu\n",
                         unsigned(indiceImpulso + 1), unsigned(numeroImpulsi),
                         (unsigned long)uint32_t(inizioStatoMs - inizioSequenzaMs),
                         (unsigned long)uint32_t(inizioStatoMs - ultimoImpulsoMs));
  }
  ultimoImpulsoMs = inizioStatoMs;             // Misura accensioni reali, non righe stampate.
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

uint16_t limitaRichiestaMs(int64_t richiestaMs) {
  if (richiestaMs < RICHIESTA_MINIMA_MS) return RICHIESTA_MINIMA_MS;
  if (richiestaMs > RICHIESTA_MASSIMA_MS) return RICHIESTA_MASSIMA_MS;
  return uint16_t(richiestaMs);
}

int64_t correzioneProporzionaleMs(uint32_t periodoMs, uint32_t fronti) {
  if (fronti == 0) return 0;                   // Nessun tempo medio misurabile.
  // Corr = Kp * (600 - Dt/fronti). Calcolo intero senza troncare prima la media.
  const int64_t erroreMs = int64_t(MS_PER_FRONTE) * fronti - periodoMs;
  const int64_t numeratore = erroreMs * KP_PER_MILLE;
  const int64_t denominatore = int64_t(fronti) * 1000;
  // Arrotonda al ms piu' vicino, in entrambe le direzioni.
  return (numeratore + (numeratore >= 0 ? denominatore / 2 : -denominatore / 2)) /
         denominatore;
}

void preparaSequenza(uint32_t now, uint32_t totale) {
  if (precedenteSequenzaValida) {
    ultimoPeriodoMs = uint32_t(now - precedenteSequenzaMs);
    frontiPeriodo = uint32_t(totale - totaleAllaSequenza);
    // Lento: meno gonfiaggio. Rapido: piu' gonfiaggio. Correzione proporzionale all'errore.
    const int64_t correzioneMs = correzioneProporzionaleMs(ultimoPeriodoMs, frontiPeriodo);
    tempoRichiestoMs = limitaRichiestaMs(int64_t(tempoRichiestoMs) + correzioneMs);
  }
  precedenteSequenzaMs = inizioSequenzaMs = now;
  totaleAllaSequenza = totale;
  precedenteSequenzaValida = true;

  // Il minimo di 220 ms garantisce blocco da 150 ms + almeno un mantenimento da 70 ms.
  numeroImpulsi = 1 + (tempoRichiestoMs - IMPULSO_BLOCCAGGIO_MS) / IMPULSO_MANTENIMENTO_MS;
  // Distribuisci sul Dt misurato; soltanto al primo avvio usa 600 ms.
  periodoDistribuzioneMs = ultimoPeriodoMs > 0 ? ultimoPeriodoMs : MS_PER_FRONTE;
}

void aggiornaFreno(uint32_t now) {
  bool nuovoBloccaggio = false;
  // Consuma la richiesta ISR: l'accensione viene eseguita nel ONCE di MOTOR_ON.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (bloccaggioRichiesto) {
      bloccaggioRichiesto = false;
      nuovoBloccaggio = true;
    }
  }
  if (nuovoBloccaggio) {
    indiceImpulso = 0;                        // Il fronte annulla i mantenimenti della vecchia sequenza.
    cambiaStato(MOTOR_ON);
  }

  // Una transizione esegue subito il once del nuovo stato, nello stesso loop.
  for (;;) {
    const bool once = ingressoStato;
    ingressoStato = false;

    switch (stato) {
      case PRE_GONFIAGGIO:
        if (once) {                            // ONCE: prepara una nuova prova.
          tempoRichiestoMs = limitaRichiestaMs(RICHIESTA_INIZIALE_MS);
          precedenteSequenzaValida = false;
          ultimoPeriodoMs = frontiPeriodo = 0;
          indiceImpulso = 0;
          numeroImpulsi = 2;
          periodoDistribuzioneMs = 0;
          mantenimentoAcceso = false;
          ultimoImpulsoMs = 0;
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
        if (once) attendiEncoder();            // ONCE: abilita la richiesta sul primo fronte.
        // ALWAYS: resta in attesa; la richiesta ISR e' gestita all'inizio del loop.
        break;

      case MOTOR_ON:
        if (once) {                            // ONCE: accendi prima di calcoli e log.
          mantenimentoAcceso = false;
          uint32_t frontiAlBloccaggio;
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            digitalWrite(MOTOR_PIN, HIGH);
            inizioStatoMs = millis();         // I 150 ms partono dall'accensione nel main loop.
            frontiAlBloccaggio = encoderTotale;
          }
          const uint16_t richiestaPrecedenteMs = tempoRichiestoMs;
          preparaSequenza(inizioStatoMs, frontiAlBloccaggio);
          const int16_t correzioneMs = int16_t(tempoRichiestoMs) - int16_t(richiestaPrecedenteMs);
          stampaImpulso(correzioneMs);
          now = millis();                    // Aggiorna anche un now letto prima del fronte.
        }
        // ALWAYS: i fronti si contano, senza allungare il bloccaggio.
        if (uint32_t(now - inizioStatoMs) >= IMPULSO_BLOCCAGGIO_MS) {
          cambiaStato(MANTENIMENTO);
          continue;
        }
        break;

      case MANTENIMENTO:
        if (once) {                            // ONCE: termina il blocco e attendi subito un nuovo fronte.
          indiceImpulso = 1;
          mantenimentoAcceso = false;
          inizioStatoMs = now;
          attendiEncoder();
        }
        // ALWAYS: genera i mantenimenti, poi resta qui in attesa del fronte, senza timeout.
        if (mantenimentoAcceso) {
          if (uint32_t(now - inizioStatoMs) >= IMPULSO_MANTENIMENTO_MS) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
              // Una nuova richiesta ha precedenza sullo spegnimento del mantenimento.
              if (!bloccaggioRichiesto) {
                digitalWrite(MOTOR_PIN, LOW);
                mantenimentoAcceso = false;
                ++indiceImpulso;
                inizioStatoMs = now;
              }
            }
          }
        } else if (indiceImpulso < numeroImpulsi &&
                   uint32_t(now - inizioSequenzaMs) >=
                     uint64_t(indiceImpulso) * periodoDistribuzioneMs / numeroImpulsi &&
                   uint32_t(now - ultimoImpulsoMs) >= periodoDistribuzioneMs / numeroImpulsi &&
                   uint32_t(now - inizioStatoMs) > 0) {
          // Conserva la distanza anche se il loop ha osservato una scadenza in ritardo.
          bool avvioMantenimento = false;
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (!bloccaggioRichiesto) {
              digitalWrite(MOTOR_PIN, HIGH);
              mantenimentoAcceso = true;
              inizioStatoMs = millis();       // Timestamp dell'accensione effettiva.
              avvioMantenimento = true;
            }
          }
          if (avvioMantenimento) stampaImpulso(0);
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
