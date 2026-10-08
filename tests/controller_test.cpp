#include <cassert>
#include <iostream>
#include "../sketchbook/FrenoPneumatico/FrenoPneumatico.ino"

namespace hardware {
  uint64_t timeUs = 0;
  int levels[24] = {}, modes[24] = {};
  void (*interrupt)() = nullptr;
  int interruptPin = -1, interruptMode = -1;
}
MockSerial Serial;

static void at(uint64_t ms) { hardware::timeUs = ms * 1000; }
static void tick(uint64_t ms) { at(ms); loop(); }
static void edge(uint64_t ms, bool runLoop = true) {
  at(ms);
  hardware::levels[A0] ^= 1;        // Salita e discesa vengono entrambe contate.
  hardware::interrupt();
  if (runLoop) loop();
}

static void reset(uint64_t startMs = 0) {
  Serial = MockSerial();
  for (int i = 0; i < 24; ++i) hardware::levels[i] = hardware::modes[i] = 0;
  hardware::interrupt = nullptr;
  hardware::interruptPin = hardware::interruptMode = -1;
  frontiPendenti = primoFronteMs = ultimoFronteMs = 0;
  inizioScattoMs = ultimoFronteScattoMs = ultimoLogMs = 0;
  at(startMs);
  setup();
}

static void finishPrecharge(uint64_t startMs = 0) {
  tick(startMs + PRECARICA_MS - 1);
  assert(stato == PRECARICA && hardware::levels[3] == HIGH);
  tick(startMs + PRECARICA_MS);
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
}

static void fourEdgeCycle() {
  reset(); finishPrecharge();
  edge(3000); edge(3010); edge(3020); edge(3030);
  tick(3200); tick(3330);
  assert(stato == ATTENDI_FRONTE && frontiScatto == 4);
}

static void startup_and_input_without_pullup() {
  reset();
  assert(hardware::levels[11] == HIGH && hardware::levels[6] == HIGH);
  assert(hardware::levels[4] == LOW && hardware::levels[3] == HIGH);
  assert(hardware::modes[3] == OUTPUT && hardware::modes[A0] == INPUT);
  assert(hardware::levels[A0] == LOW);
  assert(hardware::interruptPin == 14 && hardware::interruptMode == CHANGE);
  edge(1000, false); edge(1000, false);
  assert(frontiPendenti == 2);
  tick(1000);
  assert(frontiPendenti == 0 && frontiScatto == 0);
  finishPrecharge();
  tick(2800);
  assert(hardware::levels[3] == LOW && durataImpulsoMs == 200);
}

static void first_edge_and_single_pulse() {
  reset(); finishPrecharge();
  edge(3000);
  assert(stato == IMPULSO && hardware::levels[3] == HIGH);
  assert(durataImpulsoMs == 200 && frontiScatto == 1);
  edge(3100); tick(3199);
  assert(hardware::levels[3] == HIGH && frontiScatto == 2);
  edge(3200);
  assert(stato == ATTENDI_BLOCCO && hardware::levels[3] == LOW);
  edge(3300);
  assert(frontiScatto == 4 && hardware::levels[3] == LOW);
  tick(3599);
  assert(stato == ATTENDI_BLOCCO);
  tick(3600);
  assert(stato == ATTENDI_FRONTE && frontiScatto == 4);
}

static void each_edge_restarts_quiet_timeout() {
  reset(); finishPrecharge(); edge(3000); tick(3200);
  edge(3299); tick(3598);
  assert(stato == ATTENDI_BLOCCO);
  edge(3599);
  assert(stato == ATTENDI_BLOCCO && frontiScatto == 3);
  tick(3898); assert(stato == ATTENDI_BLOCCO);
  tick(3899); assert(stato == ATTENDI_FRONTE);
}

static void strict_threshold_and_previous_edge_count() {
  fourEdgeCycle(); edge(5400);       // 4 * 600 ms: uguaglianza, non ridurre.
  assert(ultimoPeriodoMs == 2400 && durataImpulsoMs == 200);
  assert(frontiScatto == 1 && hardware::levels[3] == HIGH);
  tick(5600); tick(5700); edge(6001);
  assert(ultimoPeriodoMs == 601 && durataImpulsoMs == 180);
  tick(6180); assert(hardware::levels[3] == HIGH);
  tick(6181); assert(hardware::levels[3] == LOW);

  fourEdgeCycle(); edge(5401);       // Un millisecondo oltre: ridurre di 20 ms.
  assert(ultimoPeriodoMs == 2401 && durataImpulsoMs == 180);
  fourEdgeCycle(); edge(3600);       // Ciclo piu' rapido: lasciare 200 ms.
  assert(ultimoPeriodoMs == 600 && durataImpulsoMs == 200);
}

static void batched_edges_keep_first_timestamp() {
  reset(); finishPrecharge();
  edge(3000, false); edge(3010, false); edge(3020, false); tick(3050);
  assert(frontiScatto == 3 && inizioScattoMs == 3000);
  assert(hardware::levels[3] == HIGH);
  tick(3249); assert(hardware::levels[3] == HIGH);
  tick(3250); assert(hardware::levels[3] == LOW);
  tick(3320); assert(stato == ATTENDI_FRONTE);
  edge(4800, false); tick(4805);
  assert(ultimoPeriodoMs == 1800 && durataImpulsoMs == 200);
}

static void continuous_motion_does_not_repeat_pump() {
  reset(); finishPrecharge();
  for (uint64_t ms = 3000; ms <= 6000; ms += 100) {
    edge(ms);
    if (ms >= 3200) assert(hardware::levels[3] == LOW);
  }
  assert(stato == ATTENDI_BLOCCO && frontiScatto == 31);
  assert(durataImpulsoMs == 200);
  tick(6299); assert(stato == ATTENDI_BLOCCO);
  tick(6300); assert(stato == ATTENDI_FRONTE);
}

static void reduction_stops_at_minimum() {
  reset(); finishPrecharge();
  const uint16_t expected[] = {200, 180, 160, 140, 120, 100, 80, 60,
                               40, 20, 20, 20, 20, 20, 20};
  for (uint64_t cycle = 0; cycle < 15; ++cycle) {
    const uint64_t start = 3000 + cycle * 601;
    edge(start);
    assert(durataImpulsoMs == expected[cycle]);
    assert(hardware::levels[3] == HIGH);
    tick(start + expected[cycle]);
    tick(start + TIMEOUT_BLOCCO_MS);
    assert(stato == ATTENDI_FRONTE && frontiScatto == 1);
  }
  assert(durataImpulsoMs == IMPULSO_MINIMO_MS);
}

static void stop_and_restart() {
  reset(); Serial.receive("s\n"); tick(1000);
  assert(stato == FERMO && hardware::levels[3] == LOW);
  edge(5000); assert(stato == FERMO && hardware::levels[3] == LOW);
  Serial.receive("a\n"); tick(6000);
  assert(stato == PRECARICA && hardware::levels[3] == HIGH);
  Serial.receive("a\n"); tick(6500); // a durante la precarica non la prolunga.
  finishPrecharge(6000);
  edge(9000); Serial.receive("s\n"); tick(9010);
  assert(stato == FERMO && hardware::levels[3] == LOW);
  edge(10000); assert(stato == FERMO);
  Serial.receive("a\n"); tick(11000);
  assert(durataImpulsoMs == 200 && frontiScatto == 0);
}

static void millis_rollover() {
  const uint64_t wrap = UINT32_MAX;
  reset(wrap - 1000); finishPrecharge(wrap - 1000);
  reset(wrap - 5000); finishPrecharge(wrap - 5000);
  const uint64_t first = wrap - 100;
  edge(first); tick(first + 199);
  assert(hardware::levels[3] == HIGH);
  tick(first + 200); assert(hardware::levels[3] == LOW);
  tick(first + 300); assert(stato == ATTENDI_FRONTE);
  edge(first + 601);
  assert(ultimoPeriodoMs == 601 && durataImpulsoMs == 180);
}

static void congested_serial_does_not_extend_motor_on() {
  reset(); Serial.txBlocked = true; finishPrecharge();
  edge(3000);
  for (uint64_t ms = 3001; ms <= 3200; ++ms) {
    Serial.receive("a\na\na\na\n");
    tick(ms);
  }
  assert(hardware::levels[3] == LOW && stato == ATTENDI_BLOCCO);
  assert(Serial.output.empty());
}

int main() {
  startup_and_input_without_pullup();
  first_edge_and_single_pulse();
  each_edge_restarts_quiet_timeout();
  strict_threshold_and_previous_edge_count();
  batched_edges_keep_first_timestamp();
  continuous_motion_does_not_repeat_pump();
  reduction_stops_at_minimum();
  stop_and_restart();
  millis_rollover();
  congested_serial_does_not_extend_motor_on();
  std::cout << "10 gruppi di test PASS (simulazione, non validazione del prototipo)\n";
}
