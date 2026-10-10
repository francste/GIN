#include <Arduino.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>

// Parametri della prova.
constexpr uint8_t MOTOR_PIN = 3;
constexpr uint8_t ENCODER_PIN = A0;             // D14, senza pull-up interno.
constexpr uint8_t DEBUG_PIN = 12;               // PE1 sul Nano Every.
constexpr uint32_t ENCODER_HOLDOFF_US = 2000;
constexpr uint32_t PRE_GONFIAGGIO_MS = 1500;
constexpr uint32_t MOTOR_ON_TIMEOUT_MS = 200;   // Dalla fine dell'ultimo impulso di bloccaggio.
constexpr uint32_t MS_PER_FRONTE = 600;
constexpr uint32_t MS_CORR_PER_FRONTE_ATTESO = MS_PER_FRONTE; // Correzione per ogni fronte aggiuntivo, entro il limite seguente.
constexpr uint8_t MAX_FRONTI_RITARDO_BLOCCAGGIO = 1; // 1: al massimo secondo fronte; 2: al massimo terzo.
constexpr uint16_t IMPULSO_BLOCCAGGIO_MS = 150;
constexpr uint16_t IMPULSO_MANTENIMENTO_MS = 100;
constexpr uint32_t INTERVALLO_MANTENIMENTO_MS = 600; // Fra due avvii, indipendente dall'obiettivo.
static_assert(IMPULSO_BLOCCAGGIO_MS > 0 && IMPULSO_MANTENIMENTO_MS > 0 &&
              INTERVALLO_MANTENIMENTO_MS > IMPULSO_MANTENIMENTO_MS &&
              MOTOR_ON_TIMEOUT_MS > 0 && MS_PER_FRONTE > 0 && MS_CORR_PER_FRONTE_ATTESO > 0,
              "Servono durate positive e una pausa fra i mantenimenti");
static_assert(MAX_FRONTI_RITARDO_BLOCCAGGIO >= 1 && MAX_FRONTI_RITARDO_BLOCCAGGIO <= 2,
              "Il ritardo massimo deve essere di uno o due fronti aggiuntivi");

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
bool recuperoAttivo = false;                 // Corr > 0 all'ingresso: un solo B, niente mantenimento.

bool bloccaggioAcceso = false;
uint32_t fronteAvvioBloccaggio = 1;           // Fronte atteso per il primo B, fissato all'ingresso.
uint32_t inizioBloccaggioMs = 0;              // Zero del log t, distinto dall'ingresso in MOTOR_ON.
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

// Quattro righe in attesa: evita di perdere Corr quando B ha appena riempito la UART.
struct RigaLog { char testo[64]; uint8_t lunghezza; };
RigaLog codaLog[4];
uint8_t primaRigaLog = 0;
uint8_t righeLogInAttesa = 0;

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
  if (lunghezza <= 0 || size_t(lunghezza) >= capacita ||
      size_t(lunghezza) >= sizeof(codaLog[0].testo) || righeLogInAttesa == 4) return;
  RigaLog &destinazione = codaLog[(primaRigaLog + righeLogInAttesa) % 4];
  memcpy(destinazione.testo, riga, size_t(lunghezza) + 1);
  destinazione.lunghezza = uint8_t(lunghezza);
  ++righeLogInAttesa;                          // Accoda la riga intera, senza scrivere sulla UART.
}

void svuotaLog() {
  while (righeLogInAttesa > 0) {
    const RigaLog &riga = codaLog[primaRigaLog];
    if (Serial.availableForWrite() < riga.lunghezza) break;
    Serial.print(riga.testo);                 // Scrive soltanto quando entra la riga intera.
    primaRigaLog = (primaRigaLog + 1) % 4;
    --righeLogInAttesa;
  }
}

void stampaBloccaggio() {
  char riga[96];
  const uint32_t distanzaMs = numeroBloccaggi == 1 ? 0 :
                              uint32_t(inizioImpulsoMs - ultimoImpulsoMs);
  const int lunghezza = snprintf(riga, sizeof(riga), "B:%lu Imp:%3u Fr:%5lu t:%10lu d:%10lu\n",
                                (unsigned long)numeroBloccaggi, unsigned(IMPULSO_BLOCCAGGIO_MS),
                                (unsigned long)frontiMotorOn,
                                (unsigned long)uint32_t(inizioImpulsoMs - inizioBloccaggioMs),
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
  const int lunghezza = snprintf(riga, sizeof(riga), "Dt:%10lu Np:%5lu Corr:%s Av:%lu\n",
                                (unsigned long)ultimoPeriodoMs,
                                (unsigned long)frontiPrecedenti, cifra, (unsigned long)fronteAvvioBloccaggio);
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
                                (unsigned long)uint32_t(inizioImpulsoMs - inizioBloccaggioMs),
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

uint32_t calcolaFronteAvvio(int64_t correzioneMs) {
  if (correzioneMs <= 0) return 1;             // Primo fronte: bloccaggio immediato.
  const uint64_t ritardo = (uint64_t(correzioneMs) + MS_CORR_PER_FRONTE_ATTESO - 1) /
                          MS_CORR_PER_FRONTE_ATTESO;
  return 1 + uint32_t(ritardo > MAX_FRONTI_RITARDO_BLOCCAGGIO
                      ? MAX_FRONTI_RITARDO_BLOCCAGGIO : ritardo);
}

void accendiBloccaggio() {
  digitalWrite(MOTOR_PIN, HIGH);
  inizioImpulsoMs = millis();
  bloccaggioAcceso = true;
  ++numeroBloccaggi;
  if (numeroBloccaggi == 1) inizioBloccaggioMs = inizioImpulsoMs;
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
  // La decisione e' congelata all'ingresso: nuovi fronti non riabilitano il mantenimento.
  durataMantenimentoMs = recuperoAttivo ? 0 : limitaDurataMs(tempoMs);
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
          recuperoAttivo = false;
          ultimoPeriodoMs = frontiPrecedenti = frontiMotorOn = 0;
          durataMotorOnMs = durataMantenimentoMs = 0;
          numeroBloccaggi = tempoBloccaggioMs = 0;
          numeroMantenimenti = indiceMantenimento = 0;
          bloccaggioAcceso = mantenimentoAcceso = false;
          fronteAvvioBloccaggio = 1;
          inizioBloccaggioMs = 0;
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
        if (once) {                            // ONCE: nuovo timer, Dt e n; decidi il fronte di avvio.
          // Salva il risultato del ciclo precedente PRIMA di azzerare i contatori.
          // Conta i mantenimenti realmente avviati, anche se interrotti da un fronte.
          const bool senzaMantenimentoPrima = precedenteSequenzaValida && indiceMantenimento == 0;
          bloccaggioAcceso = mantenimentoAcceso = false;
          numeroBloccaggi = tempoBloccaggioMs = 0;
          numeroMantenimenti = indiceMantenimento = 0;
          durataMotorOnMs = durataMantenimentoMs = 0;
          inizioSequenzaMs = inizioStatoMs = millis();
          memorizzaIntervallo(primaDeiFronti);
          recuperoAttivo = correzioneTempoMs > 0;
          fronteAvvioBloccaggio = senzaMantenimentoPrima
                                 ? calcolaFronteAvvio(correzioneTempoMs) : 1;
          // Con mantenimenti precedenti Av=1: riparti subito, senza un passaggio LOW.
          if (frontiMotorOn < fronteAvvioBloccaggio) digitalWrite(MOTOR_PIN, LOW);
        }
        // ALWAYS: prima del primo B conta tutti i fronti, senza far partire il timeout.
        if (numeroBloccaggi == 0) {
          if (frontiMotorOn >= fronteAvvioBloccaggio) {
            accendiBloccaggio();
            stampaBloccaggio();
          }
          if (once) stampaIntervallo();        // Corr positiva, zero o negativa: una volta per ciclo.
          break;
        }
        // In recupero conta i fronti, ma non riaccende ne' retriggera il timeout.
        // Consuma anche il fronte alla scadenza: non deve aprire un altro ciclo nello stesso loop.
        if (recuperoAttivo) nuovoFronte = false;
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
  primaRigaLog = righeLogInAttesa = 0;
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
  Serial.println(F("Avvio freno - Corr>0: un solo bloccaggio, niente mantenimento"));
  Serial.print(F("Ritardo massimo (fronti aggiuntivi): "));
  Serial.println(MAX_FRONTI_RITARDO_BLOCCAGGIO);
  Serial.println(F("Legenda log (tempi in ms):"));
  Serial.println(F("Dt : delta t tra ingressi MOTOR_ON successivi"));
  Serial.println(F("Np : fronti contati nel MOTOR_ON precedente"));
  Serial.println(F("Corr : correzione accumulata, sempre con segno + o -"));
  Serial.println(F("Av : numero del fronte che avvia il primo bloccaggio"));
  Serial.println(F("B : numero dell'impulso di bloccaggio nel ciclo"));
  Serial.println(F("Imp : durata dell'impulso di bloccaggio"));
  Serial.println(F("Fr : fronti contati nel MOTOR_ON attuale"));
  Serial.println(F("On : tempo totale in MOTOR_ON, incluse attesa e pause"));
  Serial.println(F("Tm : durata della finestra di mantenimento"));
  Serial.println(F("M : numero di impulsi di mantenimento previsti"));
  Serial.println(F("M:k/N : mantenimento avviato k su N previsti"));
  Serial.println(F("t : tempo dall'avvio del primo bloccaggio del ciclo"));
  Serial.println(F("d : distanza tra avvii di impulsi consecutivi"));
  Serial.println(F("s : arresta il controllo"));
  Serial.println(F("a : da fermo riavvia con pregonfiaggio"));
  cambiaStato(PRE_GONFIAGGIO);
  aggiornaFreno(millis());                    // Esegue subito il once iniziale.
}

void loop() {
  leggiComandi();                             // Lo stop viene eseguito in questo loop.
  aggiornaFreno(millis());
  svuotaLog();                                // Uscite e timer hanno precedenza sui log.
}
