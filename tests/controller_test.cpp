#include <cassert>
#include <cmath>
#include <iostream>
#include "../sketches/FrenoPneumatico/FrenoPneumatico.ino"

namespace hardware {
  uint64_t timeUs = 0;
  int levels[24] = {}, modes[24] = {};
  void (*interrupt)() = nullptr;
  int interruptPin = -1, interruptMode = -1;
}
MockSerial Serial;

static void reset() {
  brake = BrakeController();
  logPort = NonBlockingLog();
  Serial = MockSerial();
  hardware::timeUs = 0;
  encoderTotal = encoderLastMs = encoderMeanPeriodUs = 0;
  encoderPreviousUs = encoderOlderUs = encoderHistory = 0;
  setup();
}

static void at(uint32_t ms) { hardware::timeUs = uint64_t(ms) * 1000; }
static void edge(uint32_t ms) {
  at(ms);
  hardware::levels[A0] ^= 1;        // Alterna fronte salita e fronte discesa.
  hardware::interrupt();
}
static uint8_t observe(uint32_t ms) { at(ms); return brake.update(ms, readEncoder()); }

static void pins_and_both_edges() {
  reset();
  assert(hardware::levels[11] == HIGH && hardware::levels[6] == HIGH);
  assert(hardware::levels[4] == LOW && hardware::levels[3] == LOW);
  assert(hardware::modes[3] == OUTPUT && hardware::modes[A0] == INPUT_PULLUP);
  assert(hardware::interruptPin == 14 && hardware::interruptMode == CHANGE);
  // Impulsi stretti reali non sono persi da un debounce arbitrario.
  hardware::timeUs = 1000; hardware::interrupt();
  hardware::timeUs = 1100; hardware::interrupt();
  hardware::timeUs = 201000; hardware::interrupt();
  assert(readEncoder().total == 3 && readEncoder().meanPeriodUs == 100000);
  assert(!brake.pumping && !brake.armed);
}

static void minimum_and_asymmetric_encoder() {
  reset(); brake.arm(0, 0);
  edge(10); observe(10);
  edge(20); observe(20);            // Fronte vicino: la coppia non è completa.
  assert(!brake.pumping);
  edge(1210); observe(1210);        // Media 600 ms nonostante duty-cycle asimmetrico.
  assert(!brake.pumping);

  reset(); brake.arm(0, 0);
  edge(100); observe(100); edge(690); observe(690); edge(1280); observe(1280);
  assert(brake.requestedMs > 0 && brake.requestedMs < 50);
  assert(!brake.pumping && brake.sessionOnMs == 0);
}

static void count_through_delay_and_adapt() {
  reset(); brake.arm(0, 0);
  edge(100); observe(100); edge(200); observe(200); edge(300);
  assert(observe(300) & BrakeController::PUMP_ON);
  const uint16_t pulse = brake.pulseMs;
  assert(pulse >= 50 && pulse <= 200);
  edge(350); observe(350);          // Dentro l'impulso.
  edge(400); observe(400);
  assert(brake.burstPulses == 1);
  observe(300 + pulse);
  assert(!brake.pumping);
  edge(500); observe(500); edge(600); observe(600); // Dentro l'attesa.
  assert(!brake.pumping && brake.burstPulses == 1);
  assert(brake.burstEdges == 7);
  assert(observe(2400) & BrakeController::CYCLE);
  assert(brake.cycleEdges == 7 && brake.cycleMs == 2400);
  assert(std::fabs(brake.filteredError - 0.75f) < 0.001f);
  assert(brake.biasMs > 25 && !brake.moving);
}

static void mean_includes_dwell() {
  reset(); brake.arm(0, 0);
  edge(100); observe(100); edge(200); observe(200); edge(300); observe(300);
  observe(500); edge(600); observe(600);
  assert(observe(2400) & BrakeController::CYCLE);
  // Quattro fronti/2400 ms rispettano il target anche se nello scatto sono rapidi.
  assert(brake.cycleEdges == 4 && brake.filteredError == 0 && brake.biasMs == 25);
  edge(6000); observe(6000); edge(6200); observe(6200);
  assert(observe(8000) & BrakeController::CYCLE);
  assert(brake.cycleEdges == 2 && brake.cycleMs == 5600);
  assert(brake.filteredError < 0 && brake.biasMs < 25);
}

static void continuous_motion_and_fault() {
  reset(); brake.arm(0, 0);
  bool sawFault = false;
  for (uint32_t ms = 100; ms <= 6000; ms += 100) {
    edge(ms);
    if (observe(ms) & BrakeController::FAULT) { sawFault = true; break; }
  }
  assert(sawFault && brake.fault == BrakeController::TOO_MANY_PULSES);
  assert(!brake.armed && !brake.pumping);

  reset(); brake.config.maxPulsesPerBurst = 100;
  brake.config.maxSessionOnMs = 30000; brake.arm(0, 0);
  bool sawRate = false;
  for (uint32_t ms = 100; ms <= 6000; ms += 100) {
    edge(ms);
    if (observe(ms) & BrakeController::RATE) sawRate = true;
  }
  assert(sawRate && brake.sampleEdges == 60 && brake.sampleMs == 6000);
  assert(brake.biasMs > 25);
  observe(6500);
  const float bias = brake.biasMs;
  assert(observe(7800) & BrakeController::CYCLE);
  assert(brake.cycleEdges == 60 && brake.sampleEdges == 0 && brake.sampleMs == 1800);
  assert(brake.biasMs <= bias + 5); // Il recupero non riconta i 60 fronti.

  reset(); brake.config.maxSessionOnMs = 50; brake.arm(0, 0);
  edge(100); observe(100); edge(200); observe(200); edge(300);
  assert(observe(300) & BrakeController::FAULT);
  assert(brake.fault == BrakeController::ON_TIME_LIMIT && !brake.pumping);
}

static void stop_manual_and_rollover() {
  reset();
  assert(!brake.manualPulse(0, 49));
  assert(brake.manualPulse(0, 50));
  observe(49); assert(brake.pumping);
  observe(50); assert(!brake.pumping);
  assert(!brake.manualPulse(849, 50));
  assert(brake.manualPulse(850, 50));
  brake.stop(860); assert(!brake.pumping && !brake.armed);
  brake.arm(860, readEncoder().total);
  edge(900); observe(900); edge(950); observe(950); edge(1000); observe(1000);
  assert(!brake.pumping); // Riarmare non cancella l'attesa pneumatica.

  BrakeController rollover;
  const uint32_t start = UINT32_MAX - 20;
  assert(rollover.manualPulse(start, 50));
  rollover.update(uint32_t(start + 49), EncoderReading{0, 0, 0});
  assert(rollover.pumping);
  rollover.update(uint32_t(start + 50), EncoderReading{0, 0, 0});
  assert(!rollover.pumping);
  BrakeController counterRollover;
  counterRollover.arm(0, UINT32_MAX - 1);
  counterRollover.update(100, EncoderReading{UINT32_MAX, 100, 100000});
  counterRollover.update(200, EncoderReading{0, 200, 100000});
  counterRollover.update(300, EncoderReading{1, 300, 100000});
  assert(counterRollover.burstEdges == 3 && counterRollover.pumping);
}

static void uart_backpressure_does_not_extend_pulse() {
  reset(); Serial.txBlocked = true;
  Serial.receive("a\n"); loop();
  assert(brake.armed);
  edge(100); loop(); edge(200); loop(); edge(300); loop();
  assert(hardware::levels[3] == HIGH);
  const uint16_t pulse = brake.pulseMs;
  // Un terminale bloccato e molte richieste di aiuto non bloccano il loop.
  for (uint32_t ms = 301; ms <= 300UL + pulse; ++ms) {
    Serial.receive("?\n?\n?\n?\n?\n?\n?\n?\n");
    at(ms); loop();
  }
  assert(!brake.pumping && hardware::levels[3] == LOW);
  Serial.receive("s\n"); loop();
  assert(!brake.armed);
}

int main() {
  pins_and_both_edges();
  minimum_and_asymmetric_encoder();
  count_through_delay_and_adapt();
  mean_includes_dwell();
  continuous_motion_and_fault();
  stop_manual_and_rollover();
  uart_backpressure_does_not_extend_pulse();
  std::cout << "7 gruppi di test PASS (simulazione, non validazione del prototipo)\n";
}
