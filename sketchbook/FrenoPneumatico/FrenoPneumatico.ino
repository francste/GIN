#include <Arduino.h>
#include <util/atomic.h>
#include <stdlib.h>
#include "Controller.h"

constexpr uint8_t MOTOR_PIN = 3;
constexpr uint8_t ENCODER_PIN = A0; // D14 sul Nano Every.
constexpr uint32_t MIN_EDGE_US = 0; // Filtro disattivo: conta anche impulsi stretti.
constexpr bool ENCODER_PULLUP = true; // false se l'ingresso ha già un pilotaggio.

// Print normalmente aspetta quando il buffer UART è pieno. Qui i log vengono
// accodati e trasmessi solo se c'è spazio, senza ritardare lo stop della pompa.
class NonBlockingLog : public Print {
 public:
  using Print::write;
  size_t write(uint8_t value) override {
    const uint16_t next = (tail + 1) % sizeof(buffer);
    if (next == head) ++dropped;
    else { buffer[tail] = value; tail = next; }
    return 1;
  }
  void drain() {
    for (uint8_t n = 0; n < 32 && head != tail && Serial.availableForWrite() > 0; ++n) {
      Serial.write(buffer[head]);
      head = (head + 1) % sizeof(buffer);
    }
    if (head == tail && dropped != 0) {
      const uint32_t lost = dropped;
      dropped = 0;
      print(F("TX_DROPPED,")); println(lost);
    }
  }
 private:
  uint8_t buffer[768];
  uint16_t head = 0;
  uint16_t tail = 0;
  uint32_t dropped = 0;
};

NonBlockingLog logPort;
BrakeController brake;
volatile uint32_t encoderTotal = 0;
volatile uint32_t encoderLastMs = 0;
volatile uint32_t encoderMeanPeriodUs = 0;
volatile uint32_t encoderPreviousUs = 0;
volatile uint32_t encoderOlderUs = 0;
volatile uint8_t encoderHistory = 0;

void encoderISR() {
  const uint32_t now = micros();
  if (encoderHistory != 0 && uint32_t(now - encoderPreviousUs) < MIN_EDGE_US) return;
  if (encoderHistory >= 2) encoderMeanPeriodUs = uint32_t(now - encoderOlderUs) / 2;
  encoderOlderUs = encoderPreviousUs;
  encoderPreviousUs = now;
  if (encoderHistory < 3) ++encoderHistory;
  encoderLastMs = millis();
  ++encoderTotal;
}

EncoderReading readEncoder() {
  EncoderReading result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    result.total = encoderTotal;
    result.lastEdgeMs = encoderLastMs;
    result.meanPeriodUs = encoderMeanPeriodUs;
  }
  return result;
}

void printHelp() {
  logPort.println(F("a=avvia, s=arresta, ?=aiuto; comandi terminati da invio"));
  logPort.println(F("Solo da fermo: t 600 (ms/fronte), d 800 (attesa ms),"));
  logPort.println(F("q 1800 (silenzio ms), b 25 (bias ms), p 50 (impulso manuale)"));
  logPort.println(F("PUMP,t_ms,durata_ms,richiesta_ms,bias_ms"));
  logPort.println(F("CYCLE,t_ms,fronti,ciclo_ms,fronti_target,errore_filtrato,bias_ms,min_intervallo_us"));
  logPort.println(F("RATE,t_ms,fronti_campione,campione_ms,errore_filtrato,bias_ms"));
  logPort.println(F("STATUS,t_ms,armato,pompa,fronti_scatto,pompa_totale_ms,bias_ms,fault,fronti_totali,last_fronte_ms"));
}

void printPump(uint32_t now) {
  logPort.print(F("PUMP,")); logPort.print(now);
  logPort.print(','); logPort.print(brake.pulseMs);
  logPort.print(','); logPort.print(brake.requestedMs, 2);
  logPort.print(','); logPort.println(brake.biasMs, 2);
}

void command(char *line) {
  const uint32_t now = millis();
  if (line[0] == 's' && line[1] == '\0') {
    brake.stop(now);
    digitalWrite(MOTOR_PIN, LOW);
    logPort.println(F("STOP"));
    return;
  }
  if (line[0] == '?' && line[1] == '\0') { printHelp(); return; }
  if (line[0] == 'a' && line[1] == '\0') {
    if (brake.armed || brake.pumping) {
      logPort.println(F("ERR: inviare s prima di iniziare un nuovo test"));
      return;
    }
    brake.arm(now, readEncoder().total);
    digitalWrite(MOTOR_PIN, LOW);
    logPort.println(F("ARMED"));
    return;
  }
  if (brake.armed || brake.pumping) {
    logPort.println(F("ERR: inviare s prima di cambiare parametri"));
    return;
  }
  char *end;
  const long value = strtol(line + 1, &end, 10);
  while (*end == ' ') ++end;
  if (end == line + 1 || *end != '\0') {
    logPort.println(F("ERR: comando o numero non valido"));
    return;
  }
  bool accepted = false;
  switch (line[0]) {
    case 't':
      if (value >= 50 && value <= 5000 && brake.config.quietMs > uint32_t(value)) {
        brake.config.targetEdgeMs = value; accepted = true;
      }
      break;
    case 'd':
      if (value >= 50 && value <= 10000) {
        brake.config.responseWaitMs = value; accepted = true;
      }
      break;
    case 'q':
      if (value > long(brake.config.targetEdgeMs) && value <= 30000) {
        brake.config.quietMs = value; accepted = true;
      }
      break;
    case 'b':
      if (value >= -long(brake.config.proportionalMs) && value <= brake.config.maxPulseMs) {
        brake.biasMs = value; accepted = true;
      }
      break;
    case 'p':
      if (value >= brake.config.minPulseMs && value <= brake.config.maxPulseMs) {
        accepted = brake.manualPulse(now, uint16_t(value));
        digitalWrite(MOTOR_PIN, brake.pumping ? HIGH : LOW);
        if (accepted) printPump(now);
      }
      break;
  }
  logPort.println(accepted ? F("OK") : F("ERR: valore fuori limite, attesa attiva o fault"));
}

void readCommands() {
  static char line[32];
  static uint8_t length = 0;
  static bool overflow = false;
  // Lettura limitata: lo spegnimento della pompa deve essere servito spesso.
  for (uint8_t n = 0; n < 16 && Serial.available(); ++n) {
    const char c = char(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      if (overflow) logPort.println(F("ERR: comando troppo lungo"));
      else if (length) { line[length] = '\0'; command(line); }
      length = 0;
      overflow = false;
    } else if (length < sizeof(line) - 1) line[length++] = c;
    else overflow = true;
  }
}

void setup() {
  // Precarica i livelli prima di commutare i pin come uscite.
  digitalWrite(MOTOR_PIN, LOW); pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(11, HIGH); pinMode(11, OUTPUT);
  digitalWrite(6, HIGH); pinMode(6, OUTPUT);
  digitalWrite(4, LOW); pinMode(4, OUTPUT);
  pinMode(ENCODER_PIN, ENCODER_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  Serial.begin(115200);
  printHelp(); // Nessuna attesa di connessione seriale e motore spento al boot.
}

void loop() {
  const uint32_t now = millis();
  const EncoderReading encoder = readEncoder();
  const uint8_t events = brake.update(now, encoder);
  digitalWrite(MOTOR_PIN, brake.pumping ? HIGH : LOW);
  if (events & BrakeController::PUMP_ON) printPump(now);
  if (events & BrakeController::PUMP_OFF) {
    logPort.print(F("OFF,")); logPort.println(now);
  }
  if (events & BrakeController::CYCLE) {
    logPort.print(F("CYCLE,")); logPort.print(now);
    logPort.print(','); logPort.print(brake.cycleEdges);
    logPort.print(','); logPort.print(brake.cycleMs);
    logPort.print(','); logPort.print(float(brake.cycleMs) / brake.config.targetEdgeMs, 2);
    logPort.print(','); logPort.print(brake.filteredError, 3);
    logPort.print(','); logPort.print(brake.biasMs, 2);
    logPort.print(','); logPort.println(brake.minPeriodUs);
  }
  if (events & BrakeController::RATE) {
    logPort.print(F("RATE,")); logPort.print(now);
    logPort.print(','); logPort.print(brake.sampleEdges);
    logPort.print(','); logPort.print(brake.sampleMs);
    logPort.print(','); logPort.print(brake.filteredError, 3);
    logPort.print(','); logPort.println(brake.biasMs, 2);
  }
  if (events & BrakeController::FAULT) {
    logPort.print(F("FAULT,")); logPort.println(uint8_t(brake.fault));
  }
  readCommands();
  logPort.drain();
  static uint32_t lastStatusMs = 0;
  if (uint32_t(now - lastStatusMs) >= 1000) {
    lastStatusMs = now;
    logPort.print(F("STATUS,")); logPort.print(now);
    logPort.print(','); logPort.print(brake.armed);
    logPort.print(','); logPort.print(brake.pumping);
    logPort.print(','); logPort.print(brake.burstEdges);
    logPort.print(','); logPort.print(brake.sessionOnMs);
    logPort.print(','); logPort.print(brake.biasMs, 2);
    logPort.print(','); logPort.print(uint8_t(brake.fault));
    logPort.print(','); logPort.print(encoder.total);
    logPort.print(','); logPort.println(encoder.lastEdgeMs);
  }
}
