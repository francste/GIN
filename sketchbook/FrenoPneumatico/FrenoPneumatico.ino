#include <Arduino.h>
#include <util/atomic.h>
#include <stdio.h>

// Parametri della prova.
constexpr uint8_t MOTOR_PIN = 3;
constexpr uint8_t ENCODER_PIN = A0;             // D14, senza pull-up interno.
constexpr uint8_t DEBUG_PIN = 12;               // PE1 sul Nano Every.
constexpr uint32_t ENCODER_HOLDOFF_US = 2000;
constexpr uint32_t PRE_GONFIAGGIO_MS = 1500;
constexpr uint32_t MOTOR_ON_TIMEOUT_MS = 200;   // Dalla fine dell'ultimo impulso di bloccaggio.
constexpr uint32_t MS_PER_FRONTE = 600;
constexpr uint16_t IMPULSO_BLOCCAGGIO_MS = 100;
constexpr uint16_t IMPULSO_MANTENIMENTO_MS = 70;
constexpr uint16_t RICHIESTA_MINIMA_MS = IMPULSO_BLOCCAGGIO_MS + IMPULSO_MANTENIMENTO_MS;
constexpr uint16_t RICHIESTA_INIZIALE_MS = RICHIESTA_MINIMA_MS; // 170 ms con questi parametri.
constexpr uint16_t KP_PER_MILLE = 1000;        // Kp = 1.
constexpr uint16_t RICHIESTA_MASSIMA_MS = 2000;
static_assert(IMPULSO_BLOCCAGGIO_MS > 0 && IMPULSO_MANTENIMENTO_MS > 0 &&
              uint32_t(IMPULSO_BLOCCAGGIO_MS) + IMPULSO_MANTENIMENTO_MS <= RICHIESTA_MASSIMA_MS &&
              MOTOR_ON_TIMEOUT_MS > 0,
              "Servono durate positive e una richiesta massima sufficiente");

enum Stato { PRE_GONFIAGGIO, ATTENDI_FRONTE, MOTOR_ON, MANTENIMENTO, FERMO };
Stato stato = FERMO;
bool ingressoStato = true;                     // ONCE soltanto dopo una transizione.
uint32_t inizioStatoMs = 0;

// L'ISR legge l'encoder. Tutte le decisioni e le uscite motore sono nel main loop.
volatile uint32_t encoderTotale = 0;
uint32_t ultimoFronteValidoUs = 0;              // Usato soltanto nell'ISR.
bool fronteValidoRicevuto = false;
uint32_t ultimoTotaleLetto = 0;                 // Fronti gia' osservati dal main loop.

uint16_t tempoRichiestoMs = RICHIESTA_INIZIALE_MS;
bool precedenteSequenzaValida = false;
bool correzioneDisponibile = false;
uint32_t precedenteSequenzaMs = 0;
uint32_t totaleAllaSequenza = 0;
uint32_t inizioSequenzaMs = 0;
uint32_t ultimoPeriodoMs = 0;                   // Dt congelato all'ingresso in MOTOR_ON.
uint32_t frontiPeriodo = 0;                    // Contati sullo stesso intervallo di Dt.

bool bloccaggioAcceso = false;
uint32_t numeroBloccaggi = 0;
uint32_t tempoBloccaggioMs = 0;                // Somma dei tempi effettivamente accesi.
uint32_t inizioImpulsoMs = 0;
uint32_t ultimoSpegnimentoMs = 0;              // Retrigger del timeout a ogni fine impulso.
uint32_t ultimoImpulsoMs = 0;                  // Per la distanza fisica d nel log.

bool mantenimentoAcceso = false;
uint16_t numeroMantenimenti = 0;
uint16_t indiceMantenimento = 0;               // Numero di mantenimenti completati.
uint32_t residuoMantenimentoMs = 0;
uint32_t periodoDistribuzioneMs = 0;

void encoderISR() {
  // Il debug copia anche i rimbalzi che il filtro scarta.
  digitalWrite(DEBUG_PIN, digitalRead(ENCODER_PIN));
  const uint32_t nowUs = micros();
  if (fronteValidoRicevuto &&
      uint32_t(nowUs - ultimoFronteValidoUs) < ENCODER_HOLDOFF_US) return;
  ultimoFronteValidoUs = nowUs;                 // I rimbalzi scartati non prolungano il holdoff.
  fronteValidoRicevuto = true;
  ++encoderTotale;                            // CHANGE: conta salita e discesa.
}

uint32_t leggiEncoder() {
  uint32_t totale = 0;
  // Solo la lettura a 32 bit deve essere atomica sul microcontrollore a 8 bit.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { totale = encoderTotale; }
  return totale;
}

void inviaLog(const char *riga, int lunghezza, size_t capacita) {
  // Non attendere mai la UART: invia soltanto righe complete che entrano nel buffer.
  if (lunghezza > 0 && size_t(lunghezza) < capacita &&
      Serial.availableForWrite() >= lunghezza) Serial.print(riga);
}

void stampaBloccaggio() {
  char riga[96];
  const uint32_t distanzaMs = numeroBloccaggi == 1 ? 0 :
                              uint32_t(inizioImpulsoMs - ultimoImpulsoMs);
  const int lunghezza = snprintf(riga, sizeof(riga), "B:%lu Imp:%3u t:%10lu d:%10lu\n",
                                (unsigned long)numeroBloccaggi, unsigned(IMPULSO_BLOCCAGGIO_MS),
                                (unsigned long)uint32_t(inizioImpulsoMs - inizioSequenzaMs),
                                (unsigned long)distanzaMs);
  ultimoImpulsoMs = inizioImpulsoMs;            // Aggiorna anche se il log e' saltato.
  inviaLog(riga, lunghezza, sizeof(riga));
}

void stampaCorrezione(int16_t correzioneMs) {
  char riga[96];
  const int lunghezza = snprintf(riga, sizeof(riga),
                                "Corr:%+5d Dt:%10lu Fr:%5lu Req:%4u On:%4lu M:%u\n",
                                int(correzioneMs), (unsigned long)ultimoPeriodoMs,
                                (unsigned long)frontiPeriodo, unsigned(tempoRichiestoMs),
                                (unsigned long)tempoBloccaggioMs, unsigned(numeroMantenimenti));
  inviaLog(riga, lunghezza, sizeof(riga));
}

void stampaMantenimento() {
  char riga[96];
  const int lunghezza = snprintf(riga, sizeof(riga), "M:%u/%u t:%10lu d:%10lu\n",
                                unsigned(indiceMantenimento + 1), unsigned(numeroMantenimenti),
                                (unsigned long)uint32_t(inizioImpulsoMs - inizioSequenzaMs),
                                (unsigned long)uint32_t(inizioImpulsoMs - ultimoImpulsoMs));
  ultimoImpulsoMs = inizioImpulsoMs;
  inviaLog(riga, lunghezza, sizeof(riga));
}

void cambiaStato(Stato nuovoStato) {
  if (nuovoStato == stato) return;
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

void accendiBloccaggio() {
  digitalWrite(MOTOR_PIN, HIGH);
  inizioImpulsoMs = millis();
  bloccaggioAcceso = true;
  ++numeroBloccaggi;
}

void memorizzaIntervallo() {
  // Come prima: tempo e fronti si riferiscono agli inizi di due episodi MOTOR_ON.
  const uint32_t totale = leggiEncoder();
  ultimoTotaleLetto = totale;
  correzioneDisponibile = precedenteSequenzaValida;
  ultimoPeriodoMs = correzioneDisponibile ? uint32_t(inizioSequenzaMs - precedenteSequenzaMs) : 0;
  frontiPeriodo = correzioneDisponibile ? uint32_t(totale - totaleAllaSequenza) : 0;
  precedenteSequenzaMs = inizioSequenzaMs;
  totaleAllaSequenza = totale;
  precedenteSequenzaValida = true;
}

void concludiBloccaggio() {
  const uint16_t precedenteRichiestaMs = tempoRichiestoMs;
  if (correzioneDisponibile) {
    tempoRichiestoMs = limitaRichiestaMs(int64_t(tempoRichiestoMs) +
                      correzioneProporzionaleMs(ultimoPeriodoMs, frontiPeriodo));
  }
  // Se il bloccaggio ha gia' consumato la richiesta, il residuo e' zero.
  residuoMantenimentoMs = tempoBloccaggioMs < tempoRichiestoMs
                          ? tempoRichiestoMs - tempoBloccaggioMs : 0;
  numeroMantenimenti = residuoMantenimentoMs / IMPULSO_MANTENIMENTO_MS;
  periodoDistribuzioneMs = ultimoPeriodoMs > 0 ? ultimoPeriodoMs : MS_PER_FRONTE;
  stampaCorrezione(int16_t(tempoRichiestoMs) - int16_t(precedenteRichiestaMs));
}

void aggiornaFreno(uint32_t now) {
  const uint32_t totale = leggiEncoder();
  bool nuovoFronte = totale != ultimoTotaleLetto;
  ultimoTotaleLetto = totale;                 // Nessuna coda di fronti durante un'accensione.

  // Ogni transizione esegue subito il ONCE del nuovo stato nello stesso loop.
  for (;;) {
    const bool once = ingressoStato;
    ingressoStato = false;
    switch (stato) {
      case PRE_GONFIAGGIO:
        if (once) {                            // ONCE: ripristina e accendi il pregonfiaggio.
          tempoRichiestoMs = limitaRichiestaMs(RICHIESTA_INIZIALE_MS);
          precedenteSequenzaValida = correzioneDisponibile = false;
          ultimoPeriodoMs = frontiPeriodo = 0;
          numeroBloccaggi = tempoBloccaggioMs = 0;
          numeroMantenimenti = indiceMantenimento = 0;
          residuoMantenimentoMs = periodoDistribuzioneMs = 0;
          bloccaggioAcceso = mantenimentoAcceso = false;
          inizioStatoMs = now;
          digitalWrite(MOTOR_PIN, HIGH);
        }
        // ALWAYS: dopo 1500 ms passa direttamente all'attesa del fronte.
        if (uint32_t(now - inizioStatoMs) >= PRE_GONFIAGGIO_MS) {
          nuovoFronte = false;                 // I fronti del pregonfiaggio non sono accodati.
          cambiaStato(ATTENDI_FRONTE);
          continue;
        }
        break;

      case ATTENDI_FRONTE:
        if (once) digitalWrite(MOTOR_PIN, LOW); // ONCE: termina il pregonfiaggio.
        // ALWAYS: al primo fronte inizia MOTOR_ON, senza un'altra attesa temporizzata.
        if (nuovoFronte) {
          nuovoFronte = false;
          cambiaStato(MOTOR_ON);
          continue;
        }
        break;

      case MOTOR_ON:
        if (once) {                            // ONCE: primo impulso, nuovo timer e Dt congelato.
          mantenimentoAcceso = false;
          numeroBloccaggi = tempoBloccaggioMs = 0;
          numeroMantenimenti = indiceMantenimento = 0;
          residuoMantenimentoMs = 0;
          accendiBloccaggio();
          inizioSequenzaMs = inizioStatoMs = inizioImpulsoMs;
          memorizzaIntervallo();
          stampaBloccaggio();
          nuovoFronte = false;
          now = millis();
        }
        // ALWAYS: un fronte durante HIGH si conta, ma non accoda un altro impulso.
        if (bloccaggioAcceso) {
          if (uint32_t(now - inizioImpulsoMs) >= IMPULSO_BLOCCAGGIO_MS) {
            digitalWrite(MOTOR_PIN, LOW);
            bloccaggioAcceso = false;
            const uint32_t durataMs = uint32_t(now - inizioImpulsoMs);
            tempoBloccaggioMs = UINT32_MAX - tempoBloccaggioMs < durataMs
                                ? UINT32_MAX : tempoBloccaggioMs + durataMs;
            ultimoSpegnimentoMs = now;         // Retrigger: 200 ms dalla fine di questo impulso.
          }
        } else if (nuovoFronte) {              // Prima del timeout: subito un altro impulso.
          accendiBloccaggio();
          stampaBloccaggio();
        } else if (uint32_t(now - ultimoSpegnimentoMs) >= MOTOR_ON_TIMEOUT_MS) {
          concludiBloccaggio();                // Correggi una sola volta, dopo l'arresto presunto.
          cambiaStato(MANTENIMENTO);
          continue;
        }
        break;

      case MANTENIMENTO:
        if (once) {                            // ONCE: prepara solo la parte residua.
          indiceMantenimento = 0;
          mantenimentoAcceso = false;
        }
        // ALWAYS: il primo fronte annulla i mantenimenti e torna subito a MOTOR_ON.
        if (nuovoFronte) {
          nuovoFronte = false;
          cambiaStato(MOTOR_ON);
          continue;
        }
        if (mantenimentoAcceso) {
          if (uint32_t(now - inizioImpulsoMs) >= IMPULSO_MANTENIMENTO_MS) {
            digitalWrite(MOTOR_PIN, LOW);
            mantenimentoAcceso = false;
            ++indiceMantenimento;
            ultimoSpegnimentoMs = now;
          }
        } else if (indiceMantenimento < numeroMantenimenti &&
                   uint32_t(now - inizioSequenzaMs) >=
                     uint64_t(indiceMantenimento + 1) * periodoDistribuzioneMs / (numeroMantenimenti + 1) &&
                   uint32_t(now - ultimoImpulsoMs) >= periodoDistribuzioneMs / (numeroMantenimenti + 1) &&
                   uint32_t(now - ultimoSpegnimentoMs) > 0) {
          // Intervalli regolari anche con loop in ritardo; nessuna distanza minima impostata.
          digitalWrite(MOTOR_PIN, HIGH);
          mantenimentoAcceso = true;
          inizioImpulsoMs = millis();
          stampaMantenimento();
        }
        // Dopo l'ultimo mantenimento rimane qui in attesa del fronte, senza timeout.
        break;

      case FERMO:
        if (once) digitalWrite(MOTOR_PIN, LOW); // ONCE: spegni; ALWAYS: attendi il comando a.
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
