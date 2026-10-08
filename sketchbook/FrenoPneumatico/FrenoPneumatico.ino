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
             MOTOR_ON, ATTENDI_IMPULSO, FERMO };
Stato stato = FERMO;
bool ingressoStato = true;                     // true soltanto dopo una transizione.
uint32_t inizioStatoMs = 0;
uint16_t tempoRichiestoMs = RICHIESTA_INIZIALE_MS;
uint32_t inizioSequenzaMs = 0;
uint16_t numeroImpulsi = 1;
uint16_t indiceImpulso = 0;                    // 0 = prima accensione della sequenza.

// Campioni presi a ogni inizio sequenza: le accensioni intermedie non li cambiano.
volatile uint32_t encoderTotale = 0;
uint32_t ultimoFronteValidoUs = 0;              // Usato soltanto nell'ISR.
bool fronteValidoRicevuto = false;             // Accetta anche il primo fronte a t=0.
uint32_t totaleLetto = 0;
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
}

uint32_t leggiEncoder() {
  uint32_t totale;
  // Copia atomica: AVR legge il contatore a 32 bit in piu' istruzioni.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { totale = encoderTotale; }
  return totale;
}

uint16_t durataImpulsoMs() {
  return indiceImpulso == 0 ? IMPULSO_BLOCCAGGIO_MS : IMPULSO_MANTENIMENTO_MS;
}

void stampaImpulso(int16_t correzioneMs) {
  char riga[64];
  // Tempi in ms. Req = totale richiesto; N = accensione attuale / numero totale.
  const int lunghezza = snprintf(riga, sizeof(riga),
                                "Imp:%3u Corr:%+3d Dt:%10lu Fr:%10lu Req:%3u N:%3u/%3u\n",
                                unsigned(durataImpulsoMs()), int(correzioneMs),
                                (unsigned long)ultimoPeriodoMs,
                                (unsigned long)frontiPeriodo, unsigned(tempoRichiestoMs),
                                unsigned(indiceImpulso + 1), unsigned(numeroImpulsi));
  // Nessuna attesa per la UART: stampa solo se entra l'intera riga.
  if (lunghezza > 0 && lunghezza < int(sizeof(riga)) &&
      Serial.availableForWrite() >= lunghezza) Serial.print(riga);
}

void cambiaStato(Stato nuovoStato) {
  if (nuovoStato == stato) return;
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

void aggiornaFreno(uint32_t now, uint32_t totale) {
  bool nuovoFronte = uint32_t(totale - totaleLetto) != 0;
  totaleLetto = totale;

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
          inizioSequenzaMs = 0;
          inizioStatoMs = now;
          digitalWrite(MOTOR_PIN, HIGH);
        }
        // ALWAYS: termina il pregonfiaggio dopo 1,5 secondi.
        if (uint32_t(now - inizioStatoMs) >= PRE_GONFIAGGIO_MS) {
          nuovoFronte = false;                 // I fronti del pregonfiaggio sono ignorati.
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
          nuovoFronte = false;                 // Non riusa fronti osservati in questa attesa.
          cambiaStato(ATTENDI_FRONTE);
          continue;
        }
        break;

      case ATTENDI_FRONTE:
        if (once) {                            // ONCE: lascia il motore spento.
          digitalWrite(MOTOR_PIN, LOW);
        }
        // ALWAYS: un nuovo fronte avvia una sequenza.
        if (nuovoFronte) {
          indiceImpulso = 0;
          cambiaStato(MOTOR_ON);
          continue;
        }
        break;

      case MOTOR_ON:
        if (once) {                            // ONCE: accendi per la durata fissa.
          int16_t correzioneMs = 0;
          if (indiceImpulso == 0) {            // Correzione solo all'inizio sequenza.
            const uint16_t richiestaPrecedenteMs = tempoRichiestoMs;
            preparaSequenza(now, totale);
            correzioneMs = int16_t(tempoRichiestoMs) - int16_t(richiestaPrecedenteMs);
          }
          inizioStatoMs = now;
          digitalWrite(MOTOR_PIN, HIGH);
          stampaImpulso(correzioneMs);
        }
        // ALWAYS: i fronti si contano, senza riavviare il timer del motore.
        nuovoFronte = false;
        if (uint32_t(now - inizioStatoMs) >= durataImpulsoMs()) {
          ++indiceImpulso;
          cambiaStato(indiceImpulso < numeroImpulsi ? ATTENDI_IMPULSO : ATTENDI_ARRESTO);
          continue;
        }
        break;

      case ATTENDI_IMPULSO:
        if (once) {                            // ONCE: spegni fra due accensioni.
          digitalWrite(MOTOR_PIN, LOW);
          inizioStatoMs = now;
        }
        // ALWAYS: avvii a 0, Dt/N, 2*Dt/N... senza accumulare gli arrotondamenti.
        // Il prodotto a 64 bit evita overflow anche con Dt molto lungo.
        if (uint32_t(now - inizioSequenzaMs) >=
              uint64_t(indiceImpulso) * ultimoPeriodoMs / numeroImpulsi &&
            uint32_t(now - inizioStatoMs) > 0) {
          cambiaStato(MOTOR_ON);
          continue;
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
  const uint32_t totale = leggiEncoder();
  aggiornaFreno(millis(), totale);             // Esegue subito il once iniziale.
}

void loop() {
  leggiComandi();                             // Lo stop viene eseguito in questo loop.
  const uint32_t totale = leggiEncoder();
  const uint32_t now = millis();
  aggiornaFreno(now, totale);
}
