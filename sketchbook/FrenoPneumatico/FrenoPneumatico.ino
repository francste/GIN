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
constexpr uint16_t IMPULSO_BLOCCAGGIO_MS = 150;
constexpr uint16_t IMPULSO_MANTENIMENTO_MS = 100;
constexpr uint32_t INTERVALLO_MANTENIMENTO_MS = 600; // Fra due avvii, indipendente dall'obiettivo.
static_assert(IMPULSO_BLOCCAGGIO_MS > 0 && IMPULSO_MANTENIMENTO_MS > 0 &&
              INTERVALLO_MANTENIMENTO_MS > IMPULSO_MANTENIMENTO_MS &&
              MOTOR_ON_TIMEOUT_MS > 0 && MS_PER_FRONTE > 0,
              "Servono durate positive e una pausa fra i mantenimenti");

enum Stato { PRE_GONFIAGGIO, ATTENDI_FRONTE, MOTOR_ON, MANTENIMENTO, FERMO };
Stato stato = FERMO;
bool ingressoStato = true;                     // ONCE soltanto dopo una transizione.
uint32_t inizioStatoMs = 0;

// L'ISR legge l'encoder. Tutte le decisioni e le uscite motore sono nel main loop.
volatile uint32_t encoderTotale = 0;
uint32_t ultimoFronteValidoUs = 0;              // Usato soltanto nell'ISR.
bool fronteValidoRicevuto = false;
uint32_t ultimoTotaleLetto = 0;                 // Fronti gia' osservati dal main loop.

bool precedenteSequenzaValida = false;
uint32_t precedenteSequenzaMs = 0;
uint32_t inizioSequenzaMs = 0;
uint32_t ultimoPeriodoMs = 0;                   // Dt congelato all'ingresso in MOTOR_ON.
uint32_t frontiPrecedenti = 0;                // n del MOTOR_ON precedente, gia' completato.
uint32_t frontiMotorOn = 0;                   // n attuale, compreso il fronte iniziale.
uint32_t totalePrimaMotorOn = 0;              // Base del conteggio: prima del fronte iniziale.
int64_t correzioneTempoMs = 0;                // t_corr accumulata, anche negativa.
uint32_t durataMotorOnMs = 0;                 // Tempo nello stato, accensioni + pause + timeout.

bool bloccaggioAcceso = false;
bool soloPrimoBloccaggio = false;             // Modalita' fissata all'ingresso in MOTOR_ON.
uint32_t numeroBloccaggi = 0;
uint32_t tempoBloccaggioMs = 0;                // Somma dei tempi effettivamente accesi.
uint32_t inizioImpulsoMs = 0;
uint32_t ultimoSpegnimentoMs = 0;              // Retrigger del timeout a ogni fine impulso.
uint32_t ultimoImpulsoMs = 0;                  // Per la distanza fisica d nel log.

bool mantenimentoAcceso = false;
uint32_t numeroMantenimenti = 0;
uint32_t indiceMantenimento = 0;               // Numero di mantenimenti avviati.
uint32_t durataMantenimentoMs = 0;            // t_m: durata della finestra, non somma degli HIGH.
uint32_t inizioMantenimentoMs = 0;

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

void stampaIntervallo() {
  // snprintf AVR non supporta gli interi a 64 bit: converti t_corr in testo.
  char correzione[22];
  char *cifra = correzione + sizeof(correzione) - 1;
  *cifra = '\0';
  uint64_t valore = uint64_t(correzioneTempoMs < 0 ? -correzioneTempoMs : correzioneTempoMs);
  do { *--cifra = char('0' + valore % 10); valore /= 10; } while (valore > 0);
  *--cifra = correzioneTempoMs < 0 ? '-' : '+';
  char riga[96];
  const int lunghezza = snprintf(riga, sizeof(riga), "Dt:%10lu Np:%5lu Corr:%s S:%u\n",
                                (unsigned long)ultimoPeriodoMs,
                                (unsigned long)frontiPrecedenti, cifra, unsigned(soloPrimoBloccaggio));
  inviaLog(riga, lunghezza, sizeof(riga));
}

void stampaDurate() {
  char riga[96];
  const int lunghezza = snprintf(riga, sizeof(riga),
                                "Fr:%5lu On:%6lu Tm:%6lu M:%lu\n",
                                (unsigned long)frontiMotorOn, (unsigned long)durataMotorOnMs,
                                (unsigned long)durataMantenimentoMs, (unsigned long)numeroMantenimenti);
  inviaLog(riga, lunghezza, sizeof(riga));
}

void stampaMantenimento() {
  char riga[96];
  const int lunghezza = snprintf(riga, sizeof(riga), "M:%lu/%lu t:%10lu d:%10lu\n",
                                (unsigned long)indiceMantenimento, (unsigned long)numeroMantenimenti,
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

uint32_t limitaDurataMs(int64_t durataMs) {
  if (durataMs < 0) return 0;
  if (durataMs > UINT32_MAX) return UINT32_MAX;
  return uint32_t(durataMs);
}

void accendiBloccaggio() {
  digitalWrite(MOTOR_PIN, HIGH);
  inizioImpulsoMs = millis();
  bloccaggioAcceso = true;
  ++numeroBloccaggi;
}

void memorizzaIntervallo(uint32_t primaDeiFronti) {
  // Chiudi il ciclo precedente prima di azzerare n per il nuovo MOTOR_ON.
  frontiPrecedenti = precedenteSequenzaValida ? frontiMotorOn : 0;
  ultimoPeriodoMs = precedenteSequenzaValida
                    ? uint32_t(inizioSequenzaMs - precedenteSequenzaMs) : 0;
  if (precedenteSequenzaValida) {
    correzioneTempoMs += int64_t(ultimoPeriodoMs) - int64_t(frontiPrecedenti) * MS_PER_FRONTE;
    // Limite numerico dei timer a 32 bit, senza il vecchio limite di richiesta a 2000 ms.
    if (correzioneTempoMs > int64_t(UINT32_MAX)) correzioneTempoMs = UINT32_MAX;
    if (correzioneTempoMs < -int64_t(UINT32_MAX)) correzioneTempoMs = -int64_t(UINT32_MAX);
  }
  precedenteSequenzaMs = inizioSequenzaMs;
  precedenteSequenzaValida = true;
  totalePrimaMotorOn = primaDeiFronti;
  const uint32_t totale = leggiEncoder();
  ultimoTotaleLetto = totale;
  frontiMotorOn = uint32_t(totale - totalePrimaMotorOn); // Include tutti i fronti della prima lettura.
}

void preparaMantenimento() {
  inizioMantenimentoMs = millis();
  durataMotorOnMs = uint32_t(inizioMantenimentoMs - inizioSequenzaMs);
  const int64_t tempoMs = int64_t(frontiMotorOn) * MS_PER_FRONTE -
                         durataMotorOnMs - correzioneTempoMs;
  durataMantenimentoMs = limitaDurataMs(tempoMs);
  // Primo mantenimento all'ingresso; gli altri ogni 600 ms. Soltanto impulsi completi.
  numeroMantenimenti = durataMantenimentoMs < IMPULSO_MANTENIMENTO_MS ? 0 :
                      1 + (durataMantenimentoMs - IMPULSO_MANTENIMENTO_MS) / INTERVALLO_MANTENIMENTO_MS;
  indiceMantenimento = 0;
  mantenimentoAcceso = false;
  stampaDurate();
}

void aggiornaFreno(uint32_t now) {
  const uint32_t primaDeiFronti = ultimoTotaleLetto;
  const uint32_t totale = leggiEncoder();
  bool nuovoFronte = totale != ultimoTotaleLetto;
  ultimoTotaleLetto = totale;                 // Nessuna coda di fronti durante un'accensione.
  if (stato == MOTOR_ON && !ingressoStato) frontiMotorOn = uint32_t(totale - totalePrimaMotorOn);

  // Ogni transizione esegue subito il ONCE del nuovo stato nello stesso loop.
  for (;;) {
    const bool once = ingressoStato;
    ingressoStato = false;
    switch (stato) {
      case PRE_GONFIAGGIO:
        if (once) {                            // ONCE: ripristina e accendi il pregonfiaggio.
          precedenteSequenzaValida = false;
          correzioneTempoMs = 0;
          ultimoPeriodoMs = frontiPrecedenti = frontiMotorOn = 0;
          durataMotorOnMs = durataMantenimentoMs = 0;
          numeroBloccaggi = tempoBloccaggioMs = 0;
          numeroMantenimenti = indiceMantenimento = 0;
          bloccaggioAcceso = mantenimentoAcceso = soloPrimoBloccaggio = false;
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
        if (once) {                            // ONCE: termina pregonfiaggio o mantenimento.
          digitalWrite(MOTOR_PIN, LOW);
          mantenimentoAcceso = false;
        }
        // ALWAYS: al primo fronte inizia MOTOR_ON, senza un'altra attesa temporizzata.
        if (nuovoFronte) {
          nuovoFronte = false;
          cambiaStato(MOTOR_ON);
          continue;
        }
        break;

      case MOTOR_ON:
        if (once) {                            // ONCE: primo impulso, nuovo timer e Dt congelato.
          const bool senzaMantenimentiPrima = precedenteSequenzaValida && numeroMantenimenti == 0;
          mantenimentoAcceso = false;
          numeroBloccaggi = tempoBloccaggioMs = 0;
          numeroMantenimenti = indiceMantenimento = 0;
          durataMotorOnMs = durataMantenimentoMs = 0;
          accendiBloccaggio();
          inizioSequenzaMs = inizioStatoMs = inizioImpulsoMs;
          memorizzaIntervallo(primaDeiFronti);
          // Riduci la frenata se la correzione positiva ha gia' escluso il mantenimento.
          soloPrimoBloccaggio = senzaMantenimentiPrima && correzioneTempoMs > 0;
          stampaBloccaggio();
          stampaIntervallo();                  // t_corr si aggiorna soltanto all'ingresso.
          nuovoFronte = false;
          now = millis();
        }
        // In modalita' singola i fronti aggiornano n, ma non riaccendono o passano al nuovo stato.
        if (soloPrimoBloccaggio) nuovoFronte = false;
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
          cambiaStato(MANTENIMENTO);
          continue;
        }
        break;

      case MANTENIMENTO: {
        if (once) {                            // ONCE: calcola t_m dal nuovo n e dal tempo nello stato.
          preparaMantenimento();
          now = millis();
        }
        // ALWAYS: il primo fronte annulla i mantenimenti e torna subito a MOTOR_ON.
        if (nuovoFronte) {
          nuovoFronte = false;
          cambiaStato(MOTOR_ON);
          continue;
        }
        const uint32_t trascorsoMs = uint32_t(now - inizioMantenimentoMs);
        if (trascorsoMs >= durataMantenimentoMs) {
          cambiaStato(ATTENDI_FRONTE);          // Finestra conclusa: niente altri mantenimenti.
          continue;
        }
        if (mantenimentoAcceso) {
          if (uint32_t(now - inizioImpulsoMs) >= IMPULSO_MANTENIMENTO_MS) {
            digitalWrite(MOTOR_PIN, LOW);
            mantenimentoAcceso = false;
            ultimoSpegnimentoMs = now;
          }
        } else if ((indiceMantenimento == 0 ||
                    uint32_t(now - ultimoImpulsoMs) >= INTERVALLO_MANTENIMENTO_MS) &&
                   durataMantenimentoMs - trascorsoMs >= IMPULSO_MANTENIMENTO_MS) {
          // Non recuperare scadenze arretrate con una raffica; conserva 600 ms fra avvii reali.
          digitalWrite(MOTOR_PIN, HIGH);
          mantenimentoAcceso = true;
          inizioImpulsoMs = millis();
          ++indiceMantenimento;
          stampaMantenimento();
        }
        break;
      }

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
