#include <cassert>
#include <iostream>
#include <vector>
#include "../sketchbook/FrenoPneumatico/FrenoPneumatico.ino"

namespace hardware {
  uint64_t timeUs = 0;
  int levels[24] = {}, modes[24] = {}, writes[24] = {};
  void (*interrupt)() = nullptr;
  int interruptPin = -1, interruptMode = -1;
}
MockSerial Serial;

static void at(uint64_t ms) { hardware::timeUs = ms * 1000; }
static void tick(uint64_t ms) { at(ms); loop(); }
static void edge(uint64_t ms, bool runLoop = true) {
  at(ms);
  hardware::levels[A0] ^= 1;
  hardware::interrupt();
  if (runLoop) loop();
}

static void reset(uint64_t startMs = 0) {
  Serial = MockSerial();
  for (int i = 0; i < 24; ++i) {
    hardware::levels[i] = hardware::modes[i] = hardware::writes[i] = 0;
  }
  hardware::interrupt = nullptr;
  hardware::interruptPin = hardware::interruptMode = -1;
  stato = FERMO; ingressoStato = true;
  encoderTotale = totaleLetto = 0;
  precedenteMotorOnMs = totaleAlMotorOn = ultimoLogMs = 0;
  at(startMs); setup();
}

static void finishPreinflation(uint64_t startMs = 0) {
  tick(startMs + 2499);
  assert(stato == PRE_GONFIAGGIO && hardware::levels[3] == HIGH);
  tick(startMs + 2500);
  assert(stato == ATTENDI_ARRESTO && hardware::levels[3] == LOW);
}

static void ready(uint64_t startMs = 0) {
  finishPreinflation(startMs);
  tick(startMs + 2799); assert(stato == ATTENDI_ARRESTO);
  tick(startMs + 2800); assert(stato == ATTENDI_PRIMO_FRONTE);
}

static void three_edges_between_motor_starts() {
  reset(); ready(); edge(3000);
  edge(3050); edge(3100); edge(3150);
  tick(3200); tick(3500);
  assert(stato == ATTENDI_PRIMO_FRONTE);
}

static void startup_and_input_without_pullup() {
  reset();
  assert(hardware::levels[11] == HIGH && hardware::levels[6] == HIGH);
  assert(hardware::levels[4] == LOW && hardware::levels[3] == HIGH);
  assert(hardware::modes[A0] == INPUT && hardware::levels[A0] == LOW);
  assert(hardware::interruptPin == 14 && hardware::interruptMode == CHANGE);
  edge(1000, false); edge(1000, false); tick(1000);
  assert(encoderTotale == 2 && !precedenteMotorOnValido);
  ready(); edge(3000);
  assert(durataMotorOnMs == 200 && frontiPeriodo == 0 && ultimoPeriodoMs == 0);
  assert(totaleAlMotorOn == 3);               // Esclude il pregonfiaggio dai confronti.
}

static void once_runs_only_on_state_entry() {
  reset();
  int writes = hardware::writes[3];
  tick(1000); tick(2000); tick(2499);
  assert(hardware::writes[3] == writes && inizioStatoMs == 0);
  finishPreinflation();
  writes = hardware::writes[3];
  tick(2550); tick(2600); tick(2799);
  assert(hardware::writes[3] == writes && inizioStatoMs == 2500);
  tick(2800); edge(3000);
  writes = hardware::writes[3];
  tick(3001); edge(3100); tick(3199);
  assert(hardware::writes[3] == writes && precedenteMotorOnMs == 3000);
  assert(durataMotorOnMs == 200);
  tick(3200);
  assert(hardware::writes[3] == writes + 1 && hardware::levels[3] == LOW);
  Serial.receive("s"); tick(3300);
  writes = hardware::writes[3];
  Serial.receive("s"); tick(3400);
  assert(hardware::writes[3] == writes);       // Nessun nuovo ingresso in FERMO.
}

static void movement_while_waiting_for_initial_stop() {
  reset(); finishPreinflation();
  tick(2599); edge(2600);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  assert(durataMotorOnMs == 200 && precedenteMotorOnMs == 2600);
  tick(2799); assert(hardware::levels[3] == HIGH);
  tick(2800); assert(stato == ATTENDI_ARRESTO && hardware::levels[3] == LOW);
  tick(3099); assert(stato == ATTENDI_ARRESTO);
  tick(3100); assert(stato == ATTENDI_PRIMO_FRONTE);
}

static void movement_repeats_pulse_before_quiet_timeout() {
  reset(); ready(); edge(3000);
  edge(3050); edge(3100); tick(3199);
  assert(hardware::levels[3] == HIGH);
  tick(3200); assert(stato == ATTENDI_ARRESTO && hardware::levels[3] == LOW);
  edge(3300);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  assert(ultimoPeriodoMs == 300 && frontiPeriodo == 3 && durataMotorOnMs == 220);
  tick(3520); edge(3600);
  assert(stato == MOTOR_ON && ultimoPeriodoMs == 300 && frontiPeriodo == 1);
  assert(durataMotorOnMs == 240);
  tick(3840); tick(4139); assert(stato == ATTENDI_ARRESTO);
  tick(4140); assert(stato == ATTENDI_PRIMO_FRONTE && hardware::levels[3] == LOW);
}

static void correction_uses_same_time_and_counter_window() {
  three_edges_between_motor_starts(); edge(5400);
  assert(ultimoPeriodoMs == 2400 && frontiPeriodo == 4 && durataMotorOnMs == 200);
  // Il fronte che avvia il nuovo motor-on chiude il campione precedente.
  tick(5600); tick(5900); edge(6001);
  assert(ultimoPeriodoMs == 601 && frontiPeriodo == 1 && durataMotorOnMs == 180);
  const int writes = hardware::writes[3];
  tick(6002); tick(6100); tick(6180);
  assert(durataMotorOnMs == 180 && hardware::writes[3] == writes);
  tick(6181); assert(hardware::levels[3] == LOW);

  three_edges_between_motor_starts(); edge(5401);
  assert(ultimoPeriodoMs == 2401 && frontiPeriodo == 4 && durataMotorOnMs == 180);
  three_edges_between_motor_starts(); edge(4000);
  assert(ultimoPeriodoMs == 1000 && frontiPeriodo == 4 && durataMotorOnMs == 220);
}

static void delayed_loop_measures_actual_motor_on_time() {
  reset(); ready();
  edge(3000, false); edge(3010, false); edge(3020, false); tick(3050);
  assert(precedenteMotorOnMs == 3050 && totaleAlMotorOn == 3);
  tick(3249); assert(hardware::levels[3] == HIGH);
  tick(3250); assert(hardware::levels[3] == LOW);
  tick(3550); assert(stato == ATTENDI_PRIMO_FRONTE);
  edge(3650, false); tick(3651);
  assert(ultimoPeriodoMs == 601 && frontiPeriodo == 1 && durataMotorOnMs == 180);
}

static void continuous_motion_is_served_by_separate_pulses() {
  reset(); ready();
  std::vector<uint64_t> starts, lengths;
  bool wasOn = false;
  for (uint64_t ms = 3000; ms <= 6060; ++ms) {
    if (ms <= 6000 && (ms - 3000) % 100 == 0) edge(ms);
    else tick(ms);
    const bool on = hardware::levels[3] == HIGH;
    if (on && !wasOn) starts.push_back(ms);
    if (!on && wasOn) lengths.push_back(ms - starts.back());
    wasOn = on;
  }
  const std::vector<uint64_t> expectedStarts = {3000, 3300, 3600, 3900, 4200,
                                               4500, 4900, 5300, 5700};
  const std::vector<uint64_t> expectedLengths = {200, 220, 240, 260, 280,
                                                300, 320, 340, 360};
  assert(starts == expectedStarts && lengths == expectedLengths);
  assert(ultimoPeriodoMs == 400 && frontiPeriodo == 4);
  tick(6359); assert(stato == ATTENDI_ARRESTO);
  tick(6360); assert(stato == ATTENDI_PRIMO_FRONTE);
}

static void maximum_motor_on_duration() {
  reset(); ready();
  const uint16_t expected[] = {200, 220, 240, 260, 280, 300, 320, 340,
                               360, 380, 400, 400, 400, 400, 400};
  for (uint64_t cycle = 0; cycle < 15; ++cycle) {
    const uint64_t start = 3000 + cycle * 600;
    edge(start);
    assert(durataMotorOnMs == expected[cycle]);
    edge(start + 50);
    tick(start + expected[cycle]);
    assert(hardware::levels[3] == LOW);
  }
}

static void minimum_motor_on_duration() {
  reset(); ready();
  const uint16_t expected[] = {200, 180, 160, 140, 120, 100, 80, 60,
                               40, 20, 20, 20, 20, 20, 20};
  for (uint64_t cycle = 0; cycle < 15; ++cycle) {
    const uint64_t start = 3000 + cycle * 1000;
    edge(start);
    assert(durataMotorOnMs == expected[cycle] && hardware::levels[3] == HIGH);
    tick(start + expected[cycle]);
    assert(hardware::levels[3] == LOW);
    tick(start + expected[cycle] + 300);
    assert(stato == ATTENDI_PRIMO_FRONTE);
  }
}

static void stop_and_restart() {
  reset(); Serial.receive("s\n"); tick(1000);
  assert(stato == FERMO && hardware::levels[3] == LOW);
  edge(5000); assert(stato == FERMO);
  Serial.receive("a\n"); tick(6000);
  assert(stato == PRE_GONFIAGGIO && hardware::levels[3] == HIGH);
  Serial.receive("a\n"); tick(6500); ready(6000);
  edge(9000); Serial.receive("s\n"); tick(9010);
  assert(stato == FERMO && hardware::levels[3] == LOW);
  edge(10000); assert(stato == FERMO);
  Serial.receive("a\n"); tick(11000);
  assert(durataMotorOnMs == 200 && !precedenteMotorOnValido);
  assert(frontiPeriodo == 0 && ultimoPeriodoMs == 0);
}

static void millis_rollover() {
  const uint64_t wrap = UINT32_MAX;
  reset(wrap - 1000); ready(wrap - 1000);
  reset(wrap - 5000); ready(wrap - 5000);
  const uint64_t first = wrap - 100;
  edge(first); tick(first + 199); assert(hardware::levels[3] == HIGH);
  tick(first + 200); assert(hardware::levels[3] == LOW);
  tick(first + 500); assert(stato == ATTENDI_PRIMO_FRONTE);
  edge(first + 601);
  assert(ultimoPeriodoMs == 601 && frontiPeriodo == 1 && durataMotorOnMs == 180);
}

static void encoder_counter_rollover() {
  reset(); ready();
  encoderTotale = UINT32_MAX - 2;
  totaleLetto = encoderTotale;
  edge(3000); edge(3100); edge(3150);
  tick(3200); tick(3500); edge(4000);
  assert(frontiPeriodo == 3 && ultimoPeriodoMs == 1000 && durataMotorOnMs == 220);
}

static void congested_serial_does_not_extend_motor_on() {
  reset(); Serial.txBlocked = true; ready(); edge(3000);
  for (uint64_t ms = 3001; ms <= 3200; ++ms) {
    Serial.receive("a\na\na\na\n"); tick(ms);
  }
  assert(hardware::levels[3] == LOW && stato == ATTENDI_ARRESTO);
  assert(Serial.output.empty());
}

int main() {
  startup_and_input_without_pullup();
  once_runs_only_on_state_entry();
  movement_while_waiting_for_initial_stop();
  movement_repeats_pulse_before_quiet_timeout();
  correction_uses_same_time_and_counter_window();
  delayed_loop_measures_actual_motor_on_time();
  continuous_motion_is_served_by_separate_pulses();
  minimum_motor_on_duration();
  maximum_motor_on_duration();
  stop_and_restart();
  millis_rollover();
  encoder_counter_rollover();
  congested_serial_does_not_extend_motor_on();
  std::cout << "13 gruppi di test PASS (simulazione, non validazione del prototipo)\n";
}
