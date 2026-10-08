#include <Arduino.h>
#include <util/atomic.h>
#include <stdio.h>

// Parametri della prima prova: modificare questi valori per tarare il sistema.
constexpr uint8_t MOTOR_PIN = 3;
constexpr uint8_t ENCODER_PIN = A0;            // D14 sul Nano Every.
constexpr uint32_t PRECARICA_MS = 2500;
constexpr uint16_t IMPULSO_INIZIALE_MS = 200;
constexpr uint16_t PASSO_MS = 20;
constexpr uint16_t IMPULSO_MINIMO_MS = 20;
constexpr uint32_t MS_PER_FRONTE = 600;
constexpr uint32_t TIMEOUT_BLOCCO_MS = 300;    // Silenzio encoder per blocco presunto.

enum Stato { PRECARICA, ATTENDI_FRONTE, IMPULSO, ATTENDI_BLOCCO, FERMO };
Stato stato = FERMO;
uint16_t durataImpulsoMs = IMPULSO_INIZIALE_MS;
uint32_t inizioAccensioneMs = 0;
uint32_t inizioScattoMs = 0;
uint32_t ultimoFronteScattoMs = 0;
uint32_t frontiScatto = 0;
uint32_t ultimoPeriodoMs = 0;
uint32_t ultimoLogMs = 0;

// L'interrupt registra tutti i fronti; il loop li ritira insieme.
volatile uint32_t frontiPendenti = 0;
volatile uint32_t primoFronteMs = 0;
volatile uint32_t ultimoFronteMs = 0;

struct LetturaEncoder {
  uint32_t fronti;
  uint32_t primoMs;
  uint32_t ultimoMs;
};

void encoderISR() {
  const uint32_t now = millis();
  if (frontiPendenti == 0) primoFronteMs = now;
  ultimoFronteMs = now;
  ++frontiPendenti;
}

LetturaEncoder leggiEncoder() {
  LetturaEncoder lettura;
  // Su AVR una lettura a 32 bit richiede piu' istruzioni: proteggiamo la copia.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    lettura.fronti = frontiPendenti;
    lettura.primoMs = primoFronteMs;
    lettura.ultimoMs = ultimoFronteMs;
    frontiPendenti = 0;
  }
  return lettura;
}

void avvia(uint32_t now) {
  durataImpulsoMs = IMPULSO_INIZIALE_MS;
  frontiScatto = 0;
  ultimoPeriodoMs = 0;
  inizioAccensioneMs = now;
  stato = PRECARICA;
  digitalWrite(MOTOR_PIN, HIGH);
}

void aggiornaFreno(uint32_t now, const LetturaEncoder &encoder) {
  switch (stato) {
    case PRECARICA:
      // Una sola precarica; i fronti di questa fase non fanno parte dei cicli.
      if (uint32_t(now - inizioAccensioneMs) >= PRECARICA_MS) {
        digitalWrite(MOTOR_PIN, LOW);
        stato = ATTENDI_FRONTE;
      }
      break;

    case ATTENDI_FRONTE:
      if (encoder.fronti == 0) break;

      // Dal secondo scatto confrontiamo primo fronte -> primo fronte.
      // n include il primo fronte e tutti quelli prima del blocco precedente.
      if (frontiScatto != 0) {
        ultimoPeriodoMs = uint32_t(encoder.primoMs - inizioScattoMs);
        const uint64_t tempoObiettivoMs = uint64_t(frontiScatto) * MS_PER_FRONTE;
        if (ultimoPeriodoMs > tempoObiettivoMs) {
          durataImpulsoMs = durataImpulsoMs > IMPULSO_MINIMO_MS + PASSO_MS
              ? durataImpulsoMs - PASSO_MS : IMPULSO_MINIMO_MS;
        }
      }

      inizioScattoMs = encoder.primoMs;
      ultimoFronteScattoMs = encoder.ultimoMs;
      frontiScatto = encoder.fronti;
      inizioAccensioneMs = now;
      // Se si sceglie un minimo di 0 ms, a zero osserviamo senza accendere.
      stato = durataImpulsoMs != 0 ? IMPULSO : ATTENDI_BLOCCO;
      digitalWrite(MOTOR_PIN, durataImpulsoMs != 0 ? HIGH : LOW);
      break;

    case IMPULSO:
    case ATTENDI_BLOCCO:
      // I nuovi fronti vengono contati anche a pompa accesa e durante l'attesa.
      // Non comandano altri impulsi e non prolungano quello gia' in corso.
      frontiScatto += encoder.fronti;
      if (encoder.fronti != 0) ultimoFronteScattoMs = encoder.ultimoMs;

      if (stato == IMPULSO) {
        if (uint32_t(now - inizioAccensioneMs) >= durataImpulsoMs) {
          digitalWrite(MOTOR_PIN, LOW);
          stato = ATTENDI_BLOCCO;
        }
      } else if (uint32_t(now - ultimoFronteScattoMs) >= TIMEOUT_BLOCCO_MS) {
        // Blocco presunto: conserviamo n per il prossimo primo fronte.
        stato = ATTENDI_FRONTE;
      }
      break;

    case FERMO:
      break;
  }
}

void leggiComandi() {
  // Bastano s=stop e a=nuova prova. La lettura limitata mantiene rapido il loop.
  for (uint8_t i = 0; i < 8 && Serial.available() > 0; ++i) {
    const char comando = char(Serial.read());
    if (comando == 's') {
      digitalWrite(MOTOR_PIN, LOW);
      stato = FERMO;
    } else if (comando == 'a' && stato == FERMO) {
      avvia(millis());
    }
  }
}

void stampaStato(uint32_t now) {
  // Una riga breve al secondo, solo se entra nel buffer: la seriale non attende.
  char riga[60];
  if (uint32_t(now - ultimoLogMs) < 1000 || Serial.availableForWrite() < 60) return;
  ultimoLogMs = now;
  snprintf(riga, sizeof(riga), "S,%u,P,%u,N,%lu,T,%lu\n", unsigned(stato),
           unsigned(durataImpulsoMs), (unsigned long)frontiScatto,
           (unsigned long)ultimoPeriodoMs);
  Serial.print(riga);
}

void setup() {
  digitalWrite(MOTOR_PIN, LOW); pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(11, HIGH); pinMode(11, OUTPUT);
  digitalWrite(6, HIGH); pinMode(6, OUTPUT);
  digitalWrite(4, LOW); pinMode(4, OUTPUT);
  digitalWrite(ENCODER_PIN, LOW);
  pinMode(ENCODER_PIN, INPUT);                 // Encoder SENZA pull-up interno.
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  Serial.begin(115200);
  avvia(millis());                            // Precarica automatica all'avvio.
}

void loop() {
  const LetturaEncoder encoder = leggiEncoder();
  const uint32_t now = millis();              // Dopo la copia: mai prima dei fronti.
  aggiornaFreno(now, encoder);
  leggiComandi();
  stampaStato(now);
}
