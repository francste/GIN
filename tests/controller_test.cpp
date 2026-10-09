#include <cassert>
#include <iostream>
#include <vector>
#include "../sketchbook/FrenoPneumatico/FrenoPneumatico.ino"

namespace hardware {
  uint64_t timeUs = 0;
  int levels[24] = {}, modes[24] = {}, writes[24] = {};
  int outputLevels[24] = {};
  bool pullups[24] = {};
  void (*interrupt)() = nullptr;
  int interruptPin = -1, interruptMode = -1;
}
MockPort PORTA, PORTD;
MockSerial Serial;

static void at(uint64_t ms) { hardware::timeUs = ms * 1000; }
static void tick(uint64_t ms) { at(ms); loop(); }
static void edgeUs(uint64_t us, bool runLoop = true) {
  hardware::timeUs = us;
  hardware::levels[A0] ^= 1;
  hardware::interrupt();
  if (runLoop) loop();
}
static void edge(uint64_t ms, bool runLoop = true) { edgeUs(ms * 1000, runLoop); }

static void reset(uint64_t startMs = 0) {
  PORTA = PORTD = MockPort();
  Serial = MockSerial();
  for (int i = 0; i < 24; ++i) {
    hardware::levels[i] = hardware::modes[i] = hardware::writes[i] = 0;
    hardware::outputLevels[i] = 0; hardware::pullups[i] = false;
  }
  hardware::interrupt = nullptr;
  hardware::interruptPin = hardware::interruptMode = -1;
  stato = FERMO; ingressoStato = true;
  encoderTotale = 0;
  ultimoTotaleLetto = 0;
  ultimoFronteValidoUs = 0; fronteValidoRicevuto = false;
  precedenteSequenzaMs = totaleAllaSequenza = 0;
  at(startMs); setup();
}

static void ready(uint64_t startMs = 0) {
  tick(startMs + 1499);
  assert(stato == PRE_GONFIAGGIO && hardware::levels[3] == HIGH);
  tick(startMs + 1500);
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
}

struct Trace {
  std::vector<uint64_t> starts, lengths;
};

// Osserva le uscite ogni ms; non ricava le accensioni dai contatori del firmware.
static Trace observe(uint64_t from, uint64_t until,
                     const std::vector<uint64_t>& edges = {}, bool floodSerial = false) {
  Trace trace;
  bool wasOn = false;
  size_t nextEdge = 0;
  for (uint64_t ms = from; ms <= until; ++ms) {
    if (floodSerial) Serial.receive("a\na\na\na\n");
    if (nextEdge < edges.size() && edges[nextEdge] == ms) {
      edge(ms); ++nextEdge;
    } else tick(ms);
    const bool on = hardware::levels[3] == HIGH;
    if (on && !wasOn) trace.starts.push_back(ms);
    if (!on && wasOn) trace.lengths.push_back(ms - trace.starts.back());
    wasOn = on;
  }
  assert(nextEdge == edges.size());
  return trace;
}

static size_t occurrences(const std::string& text, const std::string& part) {
  size_t result = 0, pos = 0;
  while ((pos = text.find(part, pos)) != std::string::npos) { ++result; pos += part.size(); }
  return result;
}

static void first_episode(unsigned extraEdges = 0) {
  reset(); ready(); edge(3000);
  for (unsigned i = 0; i < extraEdges; ++i) edge(3002 + i * 2);
  tick(3099); assert(hardware::levels[3] == HIGH);
  tick(3100); assert(hardware::levels[3] == LOW && stato == MOTOR_ON);
  tick(3299); assert(stato == MOTOR_ON);
  tick(3300); assert(stato == MANTENIMENTO && hardware::levels[3] == HIGH);
  tick(3370); assert(hardware::levels[3] == LOW);
}

// Secondo episodio: Dt=2000, n=2, Corr=-400 applicata soltanto al timeout.
static void example_start() {
  first_episode(1); tempoRichiestoMs = 710; Serial.output.clear(); edge(5000);
  assert(ultimoPeriodoMs == 2000 && frontiPeriodo == 2 && tempoRichiestoMs == 710);
}
static void example_maintenance() {
  example_start(); tick(5100); tick(5299);
  assert(stato == MOTOR_ON && tempoRichiestoMs == 710);
  tick(5300);
  assert(stato == MANTENIMENTO && tempoRichiestoMs == 310);
  assert(tempoBloccaggioMs == 100 && residuoMantenimentoMs == 210 && numeroMantenimenti == 3);
}

static void startup_pins_and_defaults() {
  reset();
  assert(IMPULSO_BLOCCAGGIO_MS == 100 && IMPULSO_MANTENIMENTO_MS == 70);
  assert(MOTOR_ON_TIMEOUT_MS == 200 && PRE_GONFIAGGIO_MS == 1500);
  assert(KP_PER_MILLE == 1000 && RICHIESTA_MASSIMA_MS == 2000);
  assert(RICHIESTA_MINIMA_MS == 170 && tempoRichiestoMs == 170);
  assert(hardware::levels[11] == HIGH && hardware::levels[6] == HIGH);
  assert(hardware::levels[4] == LOW && hardware::levels[3] == HIGH);
  assert(hardware::modes[A0] == INPUT && !hardware::pullups[A0]);
  assert(hardware::modes[12] == OUTPUT && DEBUG_PIN == 12 && hardware::levels[12] == LOW);
  assert(PORTD.DIRCLR == PIN6_bm && PORTD.PIN6CTRL == 0);
  assert(PORTA.DIRCLR == PIN6_bm && PORTA.PIN6CTRL == 0);
  assert(hardware::interruptPin == 14 && hardware::interruptMode == CHANGE);
  assert(Serial.output == "Avvio freno\n");
}

static void pregonfiaggio_goes_directly_to_fresh_edge_wait() {
  reset(); edge(1000); edge(1498); edge(1500);
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
  assert(!precedenteSequenzaValida && encoderTotale == 3);
  tick(1501); assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
  edge(1502);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  assert(inizioSequenzaMs == 1502 && totaleAllaSequenza == 4);
  assert(ultimoPeriodoMs == 0 && frontiPeriodo == 0);
}

static void debug_and_fixed_holdoff() {
  reset(); edgeUs(0, false);
  assert(encoderTotale == 1 && hardware::levels[12] == HIGH);
  edgeUs(1, false); assert(encoderTotale == 1 && hardware::levels[12] == LOW);
  edgeUs(1999, false); assert(encoderTotale == 1 && hardware::levels[12] == HIGH);
  edgeUs(2000, false); assert(encoderTotale == 2 && hardware::levels[12] == LOW);
  edgeUs(3999, false); assert(encoderTotale == 2 && hardware::levels[12] == HIGH);
  edgeUs(4000, false); assert(encoderTotale == 3 && hardware::levels[12] == LOW);
}

static void micros_rollover_holdoff() {
  reset(); const uint64_t start = uint64_t(UINT32_MAX) - 1000;
  edgeUs(start, false); edgeUs(start + 1999, false);
  assert(encoderTotale == 1);
  edgeUs(start + 2000, false); assert(encoderTotale == 2);
}

static void isr_only_counts_and_main_loop_starts_full_pulse() {
  reset(); ready(); const int writesBefore = hardware::writes[3];
  edge(3000, false); edge(3010, false); edge(3020, false);
  assert(encoderTotale == 3 && hardware::levels[3] == LOW && hardware::writes[3] == writesBefore);
  assert(stato == ATTENDI_FRONTE && !precedenteSequenzaValida);
  tick(3250);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && inizioSequenzaMs == 3250);
  assert(totaleAllaSequenza == 3 && numeroBloccaggi == 1);
  tick(3349); assert(hardware::levels[3] == HIGH);
  tick(3350); assert(hardware::levels[3] == LOW && tempoBloccaggioMs == 100);
  tick(3549); assert(stato == MOTOR_ON);
  tick(3550); assert(stato == MANTENIMENTO && numeroBloccaggi == 1);
}

static void once_and_always_gpio_writes() {
  reset(); const int pregWrites = hardware::writes[3];
  tick(1000); tick(1499); assert(hardware::writes[3] == pregWrites);
  ready(); const int offWrites = hardware::writes[3];
  tick(1800); tick(2000); assert(hardware::writes[3] == offWrites);
  edge(3000); const int blockWrites = hardware::writes[3];
  tick(3001); edge(3010); tick(3099); assert(hardware::writes[3] == blockWrites);
  tick(3100); const int waitWrites = hardware::writes[3];
  tick(3200); tick(3299); assert(hardware::writes[3] == waitWrites);
  tick(3300); assert(hardware::writes[3] == waitWrites + 1);
  tick(3301); tick(3369); assert(hardware::writes[3] == waitWrites + 1);
  tick(3370); const int endWrites = hardware::writes[3];
  tick(9000); assert(hardware::writes[3] == endWrites && stato == MANTENIMENTO);
  Serial.receive("s"); tick(9010); const int stopWrites = hardware::writes[3];
  Serial.receive("s"); tick(9020); assert(hardware::writes[3] == stopWrites);
}

static void first_pulse_timeout_and_correction_order() {
  reset(); ready(); Serial.output.clear(); edge(3000);
  assert(Serial.output == "B:1 Imp:100 t:         0 d:         0\n");
  tick(3099); assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  tick(3100); assert(stato == MOTOR_ON && hardware::levels[3] == LOW && ultimoSpegnimentoMs == 3100);
  tick(3299); assert(occurrences(Serial.output, "Corr:") == 0);
  tick(3300);
  assert(stato == MANTENIMENTO && tempoBloccaggioMs == 100 && numeroMantenimenti == 1);
  assert(Serial.output == "B:1 Imp:100 t:         0 d:         0\n"
                          "Corr:   +0 Dt:         0 Fr:    0 Req: 170 On: 100 M:1\n"
                          "M:1/1 t:       300 d:       300\n");
  tick(3370); tick(9000); assert(occurrences(Serial.output, "Corr:") == 1);
}

static void encoder_during_high_is_counted_without_queue() {
  reset(); ready(); edge(3000);
  const Trace trace = observe(3000, 3370, {3005, 3010, 3080, 3098, 3100});
  assert(trace.starts == std::vector<uint64_t>({3000, 3300}));
  assert(trace.lengths == std::vector<uint64_t>({100, 70}));
  assert(numeroBloccaggi == 1 && tempoBloccaggioMs == 100 && encoderTotale == 6);
  assert(totaleAllaSequenza == 1 && frontiPeriodo == 0);
}

static void two_to_four_blocking_pulses_and_no_early_correction() {
  for (uint32_t count = 2; count <= 4; ++count) {
    reset(); ready(); tempoRichiestoMs = 600; Serial.output.clear(); edge(3000);
    std::vector<uint64_t> edges, starts;
    for (uint64_t i = 0; i < count; ++i) {
      starts.push_back(3000 + i * 140);
      if (i > 0) edges.push_back(starts.back());
      edges.push_back(starts.back() + 20); // Durante HIGH: conta soltanto.
    }
    const uint64_t deadline = starts.back() + 100 + 200;
    const Trace trace = observe(3000, deadline - 1, edges);
    assert(trace.starts == starts && trace.lengths == std::vector<uint64_t>(count, 100));
    assert(stato == MOTOR_ON && numeroBloccaggi == count && tempoBloccaggioMs == count * 100);
    assert(inizioSequenzaMs == 3000 && precedenteSequenzaMs == 3000);
    assert(tempoRichiestoMs == 600 && occurrences(Serial.output, "Corr:") == 0);
    tick(deadline);
    assert(stato == MANTENIMENTO && residuoMantenimentoMs == 600 - count * 100);
    assert(numeroMantenimenti == (count == 2 ? 5 : count == 3 ? 4 : 2));
    assert(occurrences(Serial.output, "Corr:") == 1);
  }
}

static void timeout_retriggered_at_end_not_start() {
  reset(); ready(); edge(3000); tick(3100); edge(3180);
  tick(3280); assert(ultimoSpegnimentoMs == 3280 && numeroBloccaggi == 2);
  tick(3300); tick(3380); tick(3479); assert(stato == MOTOR_ON);
  tick(3480);
  assert(stato == MANTENIMENTO && tempoBloccaggioMs == 200);
  assert(residuoMantenimentoMs == 0 && numeroMantenimenti == 0 && hardware::levels[3] == LOW);
}

static void fresh_edge_at_timeout_has_priority() {
  reset(); ready(); edge(3000); tick(3100); Serial.output.clear(); edge(3300);
  assert(stato == MOTOR_ON && numeroBloccaggi == 2 && hardware::levels[3] == HIGH);
  assert(occurrences(Serial.output, "Corr:") == 0);
  tick(3400); tick(3599); assert(stato == MOTOR_ON);
  tick(3600); assert(stato == MANTENIMENTO && tempoBloccaggioMs == 200);
}

static void multiple_edges_in_off_gap_start_one_pulse_in_main() {
  reset(); ready(); edge(3000); tick(3100); const int writesBefore = hardware::writes[3];
  edge(3170, false); edge(3172, false); edge(3180, false);
  assert(hardware::levels[3] == LOW && hardware::writes[3] == writesBefore);
  tick(3190); assert(hardware::levels[3] == HIGH && numeroBloccaggi == 2);
  tick(3290); tick(3490);
  assert(stato == MANTENIMENTO && numeroBloccaggi == 2 && tempoBloccaggioMs == 200);
}

static void edge_at_pulse_end_is_consumed_then_next_fresh_edge_restarts() {
  reset(); ready(); edge(3000); edge(3100);
  assert(stato == MOTOR_ON && hardware::levels[3] == LOW && numeroBloccaggi == 1);
  tick(3101); assert(hardware::levels[3] == LOW);
  edge(3102); assert(hardware::levels[3] == HIGH && numeroBloccaggi == 2);
  tick(3202); tick(3402); assert(stato == MANTENIMENTO && tempoBloccaggioMs == 200);
}

static void dt_and_front_count_frozen_until_timeout_and_next_episode() {
  example_start(); tick(5100); edge(5120); edge(5140); tick(5220); edge(5360); tick(5460);
  assert(stato == MOTOR_ON && numeroBloccaggi == 3 && tempoBloccaggioMs == 300);
  assert(ultimoPeriodoMs == 2000 && frontiPeriodo == 2 && totaleAllaSequenza == 3);
  assert(inizioSequenzaMs == 5000 && tempoRichiestoMs == 710);
  tick(5659); assert(occurrences(Serial.output, "Corr:") == 0);
  tick(5660);
  assert(stato == MANTENIMENTO && tempoRichiestoMs == 310 && numeroMantenimenti == 0);
  assert(residuoMantenimentoMs == 10 && occurrences(Serial.output, "Corr:") == 1);
  edge(7000);
  assert(stato == MOTOR_ON && ultimoPeriodoMs == 2000 && frontiPeriodo == 4);
  assert(tempoRichiestoMs == 310);
  tick(7100); tick(7300);
  assert(tempoRichiestoMs == 410 && residuoMantenimentoMs == 310 && numeroMantenimenti == 4);
}

static void all_blocking_time_subtracted_before_uniform_maintenance() {
  reset(); ready(); tempoRichiestoMs = 600; Serial.output.clear(); edge(3000);
  const Trace trace = observe(3000, 4010, {3140, 3280});
  assert(trace.starts == std::vector<uint64_t>({3000, 3140, 3280, 3580, 3700, 3820, 3940}));
  assert(trace.lengths == std::vector<uint64_t>({100, 100, 100, 70, 70, 70, 70}));
  assert(numeroBloccaggi == 3 && tempoBloccaggioMs == 300 && residuoMantenimentoMs == 300);
  assert(numeroMantenimenti == 4 && indiceMantenimento == 4);
  assert(occurrences(Serial.output, "Corr:") == 1);
  assert(Serial.output.find("M:1/4 t:       580 d:       300\n") != std::string::npos);
  assert(Serial.output.find("M:4/4 t:       940 d:       120\n") != std::string::npos);
}

static void no_maintenance_when_blocking_consumes_or_exceeds_request() {
  for (const uint16_t request : {170, 200}) {
    reset(); ready(); tempoRichiestoMs = request; edge(3000);
    const Trace trace = observe(3000, 10000, {3140});
    assert(trace.starts == std::vector<uint64_t>({3000, 3140}));
    assert(trace.lengths == std::vector<uint64_t>({100, 100}));
    assert(tempoBloccaggioMs == 200 && residuoMantenimentoMs == 0 && numeroMantenimenti == 0);
    assert(stato == MANTENIMENTO && hardware::levels[3] == LOW);
  }
}

static void blocking_may_exceed_maximum_without_unsigned_residual_underflow() {
  reset(); ready(); tempoRichiestoMs = 2000; edge(3000);
  std::vector<uint64_t> edges;
  for (uint64_t i = 1; i < 25; ++i) edges.push_back(3000 + i * 140);
  const Trace trace = observe(3000, 6660, edges);
  assert(trace.starts.size() == 25 && trace.lengths == std::vector<uint64_t>(25, 100));
  assert(tempoBloccaggioMs == 2500 && numeroMantenimenti == 0 && residuoMantenimentoMs == 0);
  assert(stato == MANTENIMENTO && tempoRichiestoMs == 2000);
}

static void actual_on_time_is_subtracted_if_loop_turns_off_late() {
  reset(); ready(); tempoRichiestoMs = 600; edge(3000); tick(3400);
  assert(tempoBloccaggioMs == 400 && hardware::levels[3] == LOW);
  tick(3599); assert(stato == MOTOR_ON);
  const Trace trace = observe(3600, 3870);
  assert(trace.starts == std::vector<uint64_t>({3600, 3800}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70}));
  assert(residuoMantenimentoMs == 200 && numeroMantenimenti == 2);
  assert(Serial.output.find("On: 400 M:2") != std::string::npos);
}

static void normal_maintenance_is_uniform_on_measured_dt() {
  example_start(); const Trace trace = observe(5000, 6570);
  assert(trace.starts == std::vector<uint64_t>({5000, 5500, 6000, 6500}));
  assert(trace.lengths == std::vector<uint64_t>({100, 70, 70, 70}));
  assert(tempoRichiestoMs == 310 && numeroMantenimenti == 3 && periodoDistribuzioneMs == 2000);
  assert(ultimoPeriodoMs == 2000 && frontiPeriodo == 2 && indiceMantenimento == 3);
  assert(occurrences(Serial.output, "Corr:") == 1);
}

static void maintenance_remains_waiting_then_new_edge_starts_episode() {
  example_start(); observe(5000, 6570); const int writesBefore = hardware::writes[3];
  tick(9000); assert(stato == MANTENIMENTO && hardware::levels[3] == LOW);
  assert(hardware::writes[3] == writesBefore);
  edge(10000, false); assert(hardware::levels[3] == LOW);
  tick(10000);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && inizioSequenzaMs == 10000);
  assert(numeroBloccaggi == 1 && tempoBloccaggioMs == 0);
}

static void maintenance_gap_is_interrupted_without_minimum_restart_distance() {
  example_maintenance(); Serial.output.clear(); edge(5400, false);
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW);
  tick(5400);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  assert(ultimoPeriodoMs == 400 && frontiPeriodo == 1 && tempoRichiestoMs == 310);
  tick(5500); tick(5699); assert(stato == MOTOR_ON);
  tick(5700);
  assert(tempoRichiestoMs == 510 && residuoMantenimentoMs == 410 && numeroMantenimenti == 5);
  assert(Serial.output.find("M:1/3") == std::string::npos);
}

static void maintenance_on_is_interrupted_without_low_glitch() {
  example_maintenance(); Serial.output.clear();
  const Trace trace = observe(5500, 6218, {5530});
  assert(trace.starts == std::vector<uint64_t>({5500, 5830, 5936, 6042, 6148}));
  assert(trace.lengths == std::vector<uint64_t>({130, 70, 70, 70, 70}));
  assert(ultimoPeriodoMs == 530 && frontiPeriodo == 1 && tempoRichiestoMs == 380);
  assert(numeroBloccaggi == 1 && tempoBloccaggioMs == 100 && numeroMantenimenti == 4);
}

static void encoder_precedes_maintenance_start_and_end_deadlines() {
  example_maintenance(); Serial.output.clear(); edge(5500);
  assert(stato == MOTOR_ON && inizioSequenzaMs == 5500 && numeroBloccaggi == 1);
  assert(Serial.output.find("M:1/3") == std::string::npos);
  example_maintenance(); tick(5500); const int writesBefore = hardware::writes[3];
  edge(5570);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  assert(hardware::writes[3] == writesBefore + 1 && inizioImpulsoMs == 5570);
  tick(5669); assert(hardware::levels[3] == HIGH);
  tick(5670); assert(hardware::levels[3] == LOW && stato == MOTOR_ON);
}

static void rounding_minimum_maximum_and_large_corrections() {
  assert(correzioneProporzionaleMs(500, 1) == 100);
  assert(correzioneProporzionaleMs(1000, 2) == 100);
  assert(correzioneProporzionaleMs(1199, 2) == 1);
  assert(correzioneProporzionaleMs(1201, 2) == -1);
  assert(correzioneProporzionaleMs(2399, 4) == 0);
  assert(correzioneProporzionaleMs(2401, 4) == 0);
  assert(correzioneProporzionaleMs(1000, 0) == 0);
  assert(correzioneProporzionaleMs(UINT32_MAX, UINT32_MAX) == 599);
  assert(correzioneProporzionaleMs(UINT32_MAX, 1) == -4294966695LL);
  assert(limitaRichiestaMs(0) == 170 && limitaRichiestaMs(2500) == 2000);
  first_episode(); tempoRichiestoMs = 2000; edge(3500); tick(3600); tick(3800);
  assert(tempoRichiestoMs == 2000 && Serial.output.find("Corr:   +0") != std::string::npos);
  first_episode(); tempoRichiestoMs = 2000; const uint64_t start = 3000ULL + UINT32_MAX;
  edge(start); tick(start + 100); tick(start + 300);
  assert(tempoRichiestoMs == 170 && residuoMantenimentoMs == 70);
}

static void proportional_correction_on_measured_samples_at_timeout() {
  const uint32_t periods[] = {3340, 2867, 3738, 2865, 4285, 4553, 5059};
  const uint32_t edges[] = {8, 14, 4, 47, 30, 29, 5};
  const int corrections[] = {183, 395, -335, 539, 457, 443, -412};
  for (size_t i = 0; i < 7; ++i) {
    first_episode(); tempoRichiestoMs = 1000; encoderTotale = totaleAllaSequenza + edges[i] - 1;
    const uint64_t start = 3000 + periods[i]; edge(start);
    assert(tempoRichiestoMs == 1000 && ultimoPeriodoMs == periods[i] && frontiPeriodo == edges[i]);
    tick(start + 100); tick(start + 299); assert(tempoRichiestoMs == 1000);
    tick(start + 300); assert(tempoRichiestoMs == 1000 + corrections[i]);
  }
}

static void maximum_request_and_residual_rounding_generate_fixed_70ms_pulses() {
  const uint16_t requests[] = {170, 200, 239, 240, 300, 2000};
  const size_t counts[] = {1, 1, 1, 2, 2, 27};
  for (size_t i = 0; i < 6; ++i) {
    reset(); ready(); tempoRichiestoMs = requests[i]; edge(3000);
    const Trace trace = observe(3000, 5500);
    assert(trace.starts.size() == counts[i] + 1);
    std::vector<uint64_t> durations(counts[i] + 1, 70); durations.front() = 100;
    assert(trace.lengths == durations && numeroMantenimenti == counts[i]);
    assert(residuoMantenimentoMs == uint32_t(requests[i] - 100) && indiceMantenimento == counts[i]);
    assert(stato == MANTENIMENTO && hardware::levels[3] == LOW);
  }
}

static void fractional_intervals_and_late_loop_preserve_spacing() {
  first_episode(3); tempoRichiestoMs = 281; edge(5003);
  const Trace fractional = observe(5003, 6675);
  assert(tempoRichiestoMs == 380 && numeroMantenimenti == 4);
  // Dt=2003, intervallo 400,6: il prodotto evita errori cumulativi.
  assert(fractional.starts == std::vector<uint64_t>({5003, 5403, 5804, 6204, 6605}));
  assert(fractional.lengths == std::vector<uint64_t>({100, 70, 70, 70, 70}));
  example_maintenance(); tick(7000);
  const Trace late = observe(7000, 8070);
  assert(late.starts == std::vector<uint64_t>({7000, 7500, 8000}));
  assert(late.lengths == std::vector<uint64_t>({70, 70, 70}));
}

static void millis_rollover_retrigger_and_counter_rollover() {
  const uint64_t wrap = uint64_t(UINT32_MAX) + 1;
  reset(wrap - 8000); ready(wrap - 8000); edge(wrap - 150);
  const Trace trace = observe(wrap - 150, wrap + 330, {wrap + 30});
  assert(trace.starts == std::vector<uint64_t>({wrap - 150, wrap + 30}));
  assert(trace.lengths == std::vector<uint64_t>({100, 100}));
  assert(stato == MANTENIMENTO && numeroMantenimenti == 0 && tempoBloccaggioMs == 200);
  assert(Serial.output.find("B:2 Imp:100 t:       180 d:       180") != std::string::npos);
  reset(); ready(); encoderTotale = UINT32_MAX - 2;
  edge(3000); edge(3010); edge(3020); tick(3100); tick(3300); tick(3370);
  tempoRichiestoMs = 700;
  edge(5000); assert(ultimoPeriodoMs == 2000 && frontiPeriodo == 3 && tempoRichiestoMs == 700);
  tick(5100); tick(5300);
  assert(tempoRichiestoMs == 633 && residuoMantenimentoMs == 533 && numeroMantenimenti == 7);
}

static void long_periods_use_64bit_distribution_products() {
  first_episode(); tempoRichiestoMs = 310; encoderTotale += 4999999;
  const uint64_t start = 3000003000ULL; edge(start); tick(start + 100); tick(start + 300);
  assert(ultimoPeriodoMs == 3000000000UL && frontiPeriodo == 5000000 && tempoRichiestoMs == 310);
  for (const uint64_t offset : {750000000ULL, 1500000000ULL, 2250000000ULL}) {
    tick(start + offset - 1); assert(hardware::levels[3] == LOW);
    tick(start + offset); assert(hardware::levels[3] == HIGH);
    tick(start + offset + 69); assert(hardware::levels[3] == HIGH);
    tick(start + offset + 70); assert(hardware::levels[3] == LOW);
  }
  assert(stato == MANTENIMENTO && indiceMantenimento == 3);
}

static void stale_loop_clock_does_not_shorten_first_pulse_and_on_sum_saturates() {
  reset(); ready(); edge(3000, false); aggiornaFreno(2999);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && inizioSequenzaMs == 3000);
  tick(3099); assert(hardware::levels[3] == HIGH);
  tempoBloccaggioMs = UINT32_MAX - 20;
  tick(3100); assert(tempoBloccaggioMs == UINT32_MAX && hardware::levels[3] == LOW);
  tick(3300); assert(numeroMantenimenti == 0 && residuoMantenimentoMs == 0);
}

static void stop_cancels_active_episode_wait_and_maintenance_then_restart_resets() {
  for (const uint64_t stopTime : {5050, 5200, 5520}) {
    example_start();
    const size_t bootMessagesBefore = occurrences(Serial.output, "Avvio freno");
    if (stopTime >= 5200) tick(5100);
    if (stopTime >= 5520) { tick(5300); tick(5500); }
    Serial.receive("s\n"); tick(stopTime);
    assert(stato == FERMO && hardware::levels[3] == LOW);
    edge(5800); tick(6000); assert(stato == FERMO && hardware::levels[3] == LOW);
    Serial.receive("a\n"); tick(7000);
    assert(stato == PRE_GONFIAGGIO && tempoRichiestoMs == 170 && !precedenteSequenzaValida);
    ready(7000); edge(9000); tick(9100); tick(9300);
    assert(stato == MANTENIMENTO && numeroBloccaggi == 1 && tempoBloccaggioMs == 100);
    assert(ultimoPeriodoMs == 0 && frontiPeriodo == 0 && numeroMantenimenti == 1);
    assert(occurrences(Serial.output, "Avvio freno") == bootMessagesBefore);
  }
  example_maintenance(); edge(5400, false); Serial.receive("s"); tick(5400);
  assert(stato == FERMO && hardware::levels[3] == LOW && precedenteSequenzaMs == 5000);
}

static void serial_congestion_and_skipped_rows_do_not_change_physical_spacing() {
  reset(); ready(); Serial.output.clear(); Serial.txBlocked = true; edge(3000);
  const Trace trace = observe(3000, 3370, {}, true);
  assert(trace.starts == std::vector<uint64_t>({3000, 3300}) && trace.lengths == std::vector<uint64_t>({100, 70}));
  assert(Serial.output.empty() && stato == MANTENIMENTO);
  example_start(); tick(5100); Serial.output.clear(); Serial.txSpace = 0;
  tick(5300); tick(5500); tick(5570); assert(Serial.output.empty());
  Serial.txSpace = 63; tick(6000);
  assert(Serial.output == "M:2/3 t:      1000 d:       500\n");
  tick(6070); tick(6500); tick(6570);
  assert(Serial.output.find("M:3/3 t:      1500 d:       500\n") != std::string::npos);
}

static void log_whole_row_required_and_large_fields_do_not_block() {
  reset(); ready(); Serial.output.clear();
  const std::string first = "B:1 Imp:100 t:         0 d:         0\n";
  Serial.txSpace = int(first.size()) - 1; edge(3000);
  assert(Serial.output.empty() && hardware::levels[3] == HIGH);
  reset(); ready(); Serial.output.clear(); Serial.txSpace = int(first.size()); edge(3000);
  assert(Serial.output == first);
  first_episode(); tempoRichiestoMs = 1401; encoderTotale = totaleAllaSequenza - 2;
  const uint64_t start = 3000ULL + UINT32_MAX;
  Serial.output.clear(); Serial.txSpace = 63; edge(start); tick(start + 100); tick(start + 300);
  assert(tempoRichiestoMs == 2000 && frontiPeriodo == UINT32_MAX && numeroMantenimenti == 27);
  assert(Serial.output.find("Dt:4294967295 Fr:4294967295") != std::string::npos);
  Serial.output.clear(); tempoBloccaggioMs = UINT32_MAX;
  stampaCorrezione(-1780); assert(Serial.output.empty());
}

int main() {
  startup_pins_and_defaults();
  pregonfiaggio_goes_directly_to_fresh_edge_wait();
  debug_and_fixed_holdoff();
  micros_rollover_holdoff();
  isr_only_counts_and_main_loop_starts_full_pulse();
  once_and_always_gpio_writes();
  first_pulse_timeout_and_correction_order();
  encoder_during_high_is_counted_without_queue();
  two_to_four_blocking_pulses_and_no_early_correction();
  timeout_retriggered_at_end_not_start();
  fresh_edge_at_timeout_has_priority();
  multiple_edges_in_off_gap_start_one_pulse_in_main();
  edge_at_pulse_end_is_consumed_then_next_fresh_edge_restarts();
  dt_and_front_count_frozen_until_timeout_and_next_episode();
  all_blocking_time_subtracted_before_uniform_maintenance();
  no_maintenance_when_blocking_consumes_or_exceeds_request();
  blocking_may_exceed_maximum_without_unsigned_residual_underflow();
  actual_on_time_is_subtracted_if_loop_turns_off_late();
  normal_maintenance_is_uniform_on_measured_dt();
  maintenance_remains_waiting_then_new_edge_starts_episode();
  maintenance_gap_is_interrupted_without_minimum_restart_distance();
  maintenance_on_is_interrupted_without_low_glitch();
  encoder_precedes_maintenance_start_and_end_deadlines();
  rounding_minimum_maximum_and_large_corrections();
  proportional_correction_on_measured_samples_at_timeout();
  maximum_request_and_residual_rounding_generate_fixed_70ms_pulses();
  fractional_intervals_and_late_loop_preserve_spacing();
  millis_rollover_retrigger_and_counter_rollover();
  long_periods_use_64bit_distribution_products();
  stale_loop_clock_does_not_shorten_first_pulse_and_on_sum_saturates();
  stop_cancels_active_episode_wait_and_maintenance_then_restart_resets();
  serial_congestion_and_skipped_rows_do_not_change_physical_spacing();
  log_whole_row_required_and_large_fields_do_not_block();
  std::cout << "33 gruppi di test PASS (simulazione, non validazione del prototipo)\n";
}
