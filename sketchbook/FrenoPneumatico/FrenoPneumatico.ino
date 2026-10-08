#include <Arduino.h>
#include <util/atomic.h>
#include <stdio.h>

// Parametri della prima prova.
constexpr uint8_t MOTOR_PIN = 3;
constexpr uint8_t ENCODER_PIN = A0;             // D14, senza pull-up interno.
constexpr uint32_t PRE_GONFIAGGIO_MS = 2500;
constexpr uint32_t TIMEOUT_ARRESTO_MS = 300;
constexpr uint32_t MS_PER_FRONTE = 600;
constexpr uint16_t IMPULSO_INIZIALE_MS = 200;
constexpr uint16_t PASSO_MS = 20;
constexpr uint16_t IMPULSO_MINIMO_MS = 20;
constexpr uint16_t IMPULSO_MASSIMO_MS = 400;

enum Stato { PRE_GONFIAGGIO, ATTENDI_ARRESTO, ATTENDI_PRIMO_FRONTE, MOTOR_ON, FERMO };
Stato stato = FERMO;
bool ingressoStato = true;                     // true soltanto dopo una transizione.
uint32_t inizioStatoMs = 0;
uint16_t durataMotorOnMs = IMPULSO_INIZIALE_MS;

// Campioni presi a ogni motor-on: tempo e contatore hanno gli stessi confini.
volatile uint32_t encoderTotale = 0;
uint32_t totaleLetto = 0;
bool precedenteMotorOnValido = false;
uint32_t precedenteMotorOnMs = 0;
uint32_t totaleAlMotorOn = 0;
uint32_t ultimoPeriodoMs = 0;
uint32_t frontiPeriodo = 0;
uint32_t ultimoLogMs = 0;

void encoderISR() {
  ++encoderTotale;                             // CHANGE: conta salita e discesa.
}

uint32_t leggiEncoder() {
  uint32_t totale;
  // Copia atomica: AVR legge il contatore a 32 bit in piu' istruzioni.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { totale = encoderTotale; }
  return totale;
}

void cambiaStato(Stato nuovoStato) {
  if (nuovoStato == stato) return;
  stato = nuovoStato;
  ingressoStato = true;
}

void correggiDurata(uint32_t now, uint32_t totale) {
  if (precedenteMotorOnValido) {
    ultimoPeriodoMs = uint32_t(now - precedenteMotorOnMs);
    frontiPeriodo = uint32_t(totale - totaleAlMotorOn);
    const uint64_t tempoObiettivoMs = uint64_t(frontiPeriodo) * MS_PER_FRONTE;

    // Lento: meno gonfiaggio. Rapido: piu' gonfiaggio. Uguale: nessuna modifica.
    int32_t nuovaDurataMs = durataMotorOnMs;
    if (ultimoPeriodoMs > tempoObiettivoMs) nuovaDurataMs -= PASSO_MS;
    else if (ultimoPeriodoMs < tempoObiettivoMs) nuovaDurataMs += PASSO_MS;
    if (nuovaDurataMs < IMPULSO_MINIMO_MS) nuovaDurataMs = IMPULSO_MINIMO_MS;
    if (nuovaDurataMs > IMPULSO_MASSIMO_MS) nuovaDurataMs = IMPULSO_MASSIMO_MS;
    durataMotorOnMs = uint16_t(nuovaDurataMs);
  }
  precedenteMotorOnMs = now;
  totaleAlMotorOn = totale;
  precedenteMotorOnValido = true;
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
          durataMotorOnMs = IMPULSO_INIZIALE_MS;
          precedenteMotorOnValido = false;
          ultimoPeriodoMs = frontiPeriodo = 0;
          inizioStatoMs = now;
          digitalWrite(MOTOR_PIN, HIGH);
        }
        // ALWAYS: termina il pregonfiaggio dopo 2,5 secondi.
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
        // ALWAYS: se scatta, frena; altrimenti conferma l'arresto col timeout.
        if (nuovoFronte) {
          cambiaStato(MOTOR_ON);
          continue;
        }
        if (uint32_t(now - inizioStatoMs) >= TIMEOUT_ARRESTO_MS) {
          cambiaStato(ATTENDI_PRIMO_FRONTE);
          continue;
        }
        break;

      case ATTENDI_PRIMO_FRONTE:
        if (once) {                            // ONCE: lascia il motore spento.
          digitalWrite(MOTOR_PIN, LOW);
        }
        // ALWAYS: il primo fronte dopo l'arresto avvia un impulso.
        if (nuovoFronte) {
          cambiaStato(MOTOR_ON);
          continue;
        }
        break;

      case MOTOR_ON:
        if (once) {                            // ONCE: correggi e accendi una volta.
          correggiDurata(now, totale);
          inizioStatoMs = now;
          digitalWrite(MOTOR_PIN, durataMotorOnMs != 0 ? HIGH : LOW);
        }
        // ALWAYS: i fronti si contano, senza riavviare il timer del motore.
        nuovoFronte = false;
        if (uint32_t(now - inizioStatoMs) >= durataMotorOnMs) {
          cambiaStato(ATTENDI_ARRESTO);
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

void stampaStato(uint32_t now) {
  // Una riga al secondo solo se entra nel buffer, senza aspettare la seriale.
  char riga[60];
  if (uint32_t(now - ultimoLogMs) < 1000 || Serial.availableForWrite() < 60) return;
  ultimoLogMs = now;
  snprintf(riga, sizeof(riga), "S,%u,P,%u,N,%lu,T,%lu\n", unsigned(stato),
           unsigned(durataMotorOnMs), (unsigned long)frontiPeriodo,
           (unsigned long)ultimoPeriodoMs);
  Serial.print(riga);
}

void setup() {
  digitalWrite(MOTOR_PIN, LOW); pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(11, HIGH); pinMode(11, OUTPUT);
  digitalWrite(6, HIGH); pinMode(6, OUTPUT);
  digitalWrite(4, LOW); pinMode(4, OUTPUT);
  digitalWrite(ENCODER_PIN, LOW); pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  Serial.begin(115200);
  cambiaStato(PRE_GONFIAGGIO);
  const uint32_t totale = leggiEncoder();
  aggiornaFreno(millis(), totale);             // Esegue subito il once iniziale.
}

void loop() {
  leggiComandi();                             // Lo stop viene eseguito in questo loop.
  const uint32_t totale = leggiEncoder();
  const uint32_t now = millis();
  aggiornaFreno(now, totale);
  stampaStato(now);
}
