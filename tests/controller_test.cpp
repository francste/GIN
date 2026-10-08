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
  encoderTotale = totaleLetto = 0;
  ultimoFronteValidoUs = 0; fronteValidoRicevuto = false;
  precedenteSequenzaMs = totaleAllaSequenza = 0;
  at(startMs); setup();
}

static void ready(uint64_t startMs = 0) {
  tick(startMs + 2499);
  assert(stato == PRE_GONFIAGGIO && hardware::levels[3] == HIGH);
  tick(startMs + 2500);
  assert(stato == ATTENDI_ARRESTO && hardware::levels[3] == LOW);
  tick(startMs + 2799); assert(stato == ATTENDI_ARRESTO);
  tick(startMs + 2800); assert(stato == ATTENDI_FRONTE);
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

static void first_sequence() {
  reset(); ready(); edge(3000);
  tick(3069); assert(hardware::levels[3] == HIGH);
  tick(3070); assert(hardware::levels[3] == LOW && stato == ATTENDI_ARRESTO);
  tick(3369); assert(stato == ATTENDI_ARRESTO);
  tick(3370); assert(stato == ATTENDI_FRONTE);
}

// Precondizione: il regolatore ha gia' raggiunto 320 ms di richiesta.
// Un periodo lento di 2000 ms la porta a 300 ms: l'esempio richiesto dall'utente.
static void example_sequence() {
  first_sequence(); tempoRichiestoMs = 320; edge(5000);
  assert(tempoRichiestoMs == 300 && ultimoPeriodoMs == 2000 && frontiPeriodo == 1);
}

static void startup_and_inputs_without_pullup() {
  reset();
  assert(IMPULSO_FISSO_MS == 70);
  assert(hardware::levels[11] == HIGH && hardware::levels[6] == HIGH);
  assert(hardware::levels[4] == LOW && hardware::levels[3] == HIGH);
  assert(hardware::modes[A0] == INPUT && !hardware::pullups[A0]);
  assert(DEBUG_PIN == 12 && hardware::modes[12] == OUTPUT && hardware::levels[12] == LOW);
  assert(PORTD.DIRCLR == PIN6_bm && PORTD.PIN6CTRL == 0);
  assert(PORTA.DIRCLR == PIN6_bm && PORTA.PIN6CTRL == 0);
  assert(hardware::interruptPin == 14 && hardware::interruptMode == CHANGE);
  edge(1000, false); edge(1000, false); tick(1000);
  assert(encoderTotale == 1 && !precedenteSequenzaValida);
  ready(); edge(3000);
  assert(tempoRichiestoMs == 200 && frontiPeriodo == 0 && ultimoPeriodoMs == 0);
  assert(numeroImpulsi == 1 && totaleAllaSequenza == 2);
  tick(3070); tick(3370); tick(10000);
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
}

static void debug_mirrors_raw_edges_and_holdoff_is_fixed() {
  reset(); edgeUs(0, false);
  assert(encoderTotale == 1 && hardware::levels[12] == HIGH);
  edgeUs(1, false);
  assert(encoderTotale == 1 && hardware::levels[12] == LOW);
  edgeUs(1999, false);
  assert(encoderTotale == 1 && hardware::levels[12] == HIGH);
  edgeUs(2000, false);
  assert(encoderTotale == 2 && hardware::levels[12] == LOW);
  edgeUs(3999, false);
  assert(encoderTotale == 2 && hardware::levels[12] == HIGH);
  edgeUs(4000, false);
  assert(encoderTotale == 3 && hardware::levels[12] == LOW);
}

static void encoder_holdoff_micros_rollover() {
  reset();
  const uint64_t firstUs = uint64_t(UINT32_MAX) - 1000;
  edgeUs(firstUs, false); edgeUs(firstUs + 1999, false);
  assert(encoderTotale == 1 && hardware::levels[12] == LOW);
  edgeUs(firstUs + 2000, false);
  assert(encoderTotale == 2 && hardware::levels[12] == HIGH);
}

static void once_runs_only_on_state_entry() {
  reset(); const int bootWrites = hardware::writes[3];
  tick(1000); tick(2000); tick(2499);
  assert(hardware::writes[3] == bootWrites && inizioStatoMs == 0);
  ready(); edge(3000);
  const int onWrites = hardware::writes[3];
  tick(3001); edge(3010); tick(3069);
  assert(hardware::writes[3] == onWrites && precedenteSequenzaMs == 3000);
  tick(3070);
  assert(hardware::writes[3] == onWrites + 1 && hardware::levels[3] == LOW);
  tick(3100); tick(3369);
  assert(hardware::writes[3] == onWrites + 1 && inizioStatoMs == 3070);
  Serial.receive("s"); tick(3400);
  const int stopWrites = hardware::writes[3];
  Serial.receive("s"); tick(3500);
  assert(hardware::writes[3] == stopWrites);
}

static void initial_wait_is_fixed_and_consumes_deadline_edge() {
  reset(); tick(2500); edge(2600); edge(2798);
  assert(stato == ATTENDI_ARRESTO && hardware::levels[3] == LOW && inizioStatoMs == 2500);
  edge(2800);
  assert(stato == ATTENDI_FRONTE && !precedenteSequenzaValida);
  tick(2801); assert(hardware::levels[3] == LOW);
  edge(2802);
  assert(stato == MOTOR_ON && precedenteSequenzaMs == 2802 && totaleAllaSequenza == 4);
  tick(2871); assert(hardware::levels[3] == HIGH);
  tick(2872); assert(hardware::levels[3] == LOW);
}

static void example_has_four_70ms_pulses_500ms_apart() {
  example_sequence();
  const Trace trace = observe(5000, 6870);
  assert(trace.starts == std::vector<uint64_t>({5000, 5500, 6000, 6500}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70, 70, 70}));
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
  tick(10000); assert(hardware::levels[3] == LOW); // Nessun riavvio automatico.
}

static void edges_do_not_restart_sequence_and_remain_in_next_sample() {
  example_sequence();
  const Trace trace = observe(5000, 6870, {5020, 5200, 5550, 6100, 6700, 6870});
  assert(trace.starts == std::vector<uint64_t>({5000, 5500, 6000, 6500}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70, 70, 70}));
  assert(precedenteSequenzaMs == 5000 && frontiPeriodo == 1 && tempoRichiestoMs == 300);
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
  tick(6871); assert(hardware::levels[3] == LOW);
  edge(6872);
  assert(precedenteSequenzaMs == 6872 && ultimoPeriodoMs == 1872);
  assert(frontiPeriodo == 7 && tempoRichiestoMs == 320 && hardware::levels[3] == HIGH);
}

static void correction_uses_same_sequence_time_and_counter_window() {
  first_sequence(); edge(3500); // T=500, n=1: richiesta 220, tre accensioni.
  const Trace trace = observe(3500, 4203);
  assert(trace.starts == std::vector<uint64_t>({3500, 3666, 3833}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70, 70}));
  assert(precedenteSequenzaMs == 3500 && tempoRichiestoMs == 220);
  edge(4204); // T=704, n=1: richiesta 200, due accensioni.
  assert(ultimoPeriodoMs == 704 && frontiPeriodo == 1 && tempoRichiestoMs == 200);

  first_sequence(); edge(3600); // T=600, n=1: invariata.
  assert(ultimoPeriodoMs == 600 && tempoRichiestoMs == 200 && numeroImpulsi == 2);
  first_sequence(); edge(3601); // T=601, n=1: -20.
  assert(ultimoPeriodoMs == 601 && tempoRichiestoMs == 180 && numeroImpulsi == 2);
}

static void fractional_spacing_does_not_accumulate_rounding() {
  first_sequence(); tempoRichiestoMs = 230; edge(5000);
  assert(tempoRichiestoMs == 210 && numeroImpulsi == 3);
  const Trace trace = observe(5000, 6703);
  assert(trace.starts == std::vector<uint64_t>({5000, 5666, 6333}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70, 70}));
  assert(stato == ATTENDI_FRONTE);
}

static void delayed_loop_measures_actual_sequence_start() {
  reset(); ready();
  edge(3000, false); edge(3010, false); edge(3020, false); tick(3050);
  assert(precedenteSequenzaMs == 3050 && totaleAllaSequenza == 3);
  tick(3119); assert(hardware::levels[3] == HIGH);
  tick(3120); assert(hardware::levels[3] == LOW);
  tick(3420); assert(stato == ATTENDI_FRONTE);
  Serial.output.clear(); edge(3650, false); tick(3651);
  assert(ultimoPeriodoMs == 601 && frontiPeriodo == 1 && tempoRichiestoMs == 180);
  assert(Serial.output == "Imp: 70 Corr:-20 Dt:       601 Fr:         1 Req:180 N:  1/  2\n");
  const Trace trace = observe(3651, 4321);
  assert(trace.starts == std::vector<uint64_t>({3651, 3951}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70}));
}

static void late_loop_does_not_merge_or_shorten_pulses() {
  example_sequence(); tick(7000); // Anche lo spegnimento viene osservato in ritardo.
  assert(stato == ATTENDI_IMPULSO && hardware::levels[3] == LOW);
  tick(7000); assert(hardware::levels[3] == LOW); // Almeno 1 ms spento.
  const Trace trace = observe(7001, 7513);
  assert(trace.starts == std::vector<uint64_t>({7001, 7072, 7143}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70, 70}));
  assert(precedenteSequenzaMs == 5000 && ultimoPeriodoMs == 2000);
  assert(stato == ATTENDI_FRONTE);
}

static void maximum_requested_time_keeps_physical_pulses_fixed() {
  reset(); ready();
  const uint16_t requests[] = {200, 220, 240, 260, 280, 300, 320, 340,
                                360, 380, 400, 400, 400, 400, 400};
  const size_t counts[] = {1, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5};
  for (uint64_t cycle = 0; cycle < 15; ++cycle) {
    const uint64_t start = 3000 + cycle * 2000;
    Serial.output.clear(); edge(start);
    assert(tempoRichiestoMs == requests[cycle]);
    if (cycle >= 11)
      assert(Serial.output == "Imp: 70 Corr: +0 Dt:      2000 Fr:         4 Req:400 N:  1/  5\n");
    const Trace trace = observe(start, start + 1999, {start + 20, start + 40, start + 60});
    assert(trace.starts.size() == counts[cycle]);
    assert(trace.lengths == std::vector<uint64_t>(counts[cycle], 70));
    assert(stato == ATTENDI_FRONTE);
  }
}

static void minimum_request_still_generates_one_fixed_pulse() {
  reset(); ready();
  const uint16_t requests[] = {200, 180, 160, 140, 120, 100, 80, 60,
                                50, 50, 50, 50, 50, 50, 50};
  const size_t counts[] = {1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  for (uint64_t cycle = 0; cycle < 15; ++cycle) {
    const uint64_t start = 3000 + cycle * 1000;
    Serial.output.clear(); edge(start);
    assert(tempoRichiestoMs == requests[cycle]);
    if (cycle == 8)
      assert(Serial.output == "Imp: 70 Corr:-10 Dt:      1000 Fr:         1 Req: 50 N:  1/  1\n");
    if (cycle >= 9)
      assert(Serial.output == "Imp: 70 Corr: +0 Dt:      1000 Fr:         1 Req: 50 N:  1/  1\n");
    const Trace trace = observe(start, start + 999);
    assert(trace.starts.size() == counts[cycle]);
    assert(trace.lengths == std::vector<uint64_t>(counts[cycle], 70));
    assert(stato == ATTENDI_FRONTE);
  }
}

static void stop_and_restart_cancel_on_pulse_and_gap() {
  for (const uint64_t stopTime : {5010, 5300}) {
    example_sequence();
    if (stopTime == 5300) {
      tick(5070);
      assert(stato == ATTENDI_IMPULSO && hardware::levels[3] == LOW);
    }
    Serial.receive("s\n"); tick(stopTime);
    assert(stato == FERMO && hardware::levels[3] == LOW);
    edge(5500); tick(6000); tick(6500);
    assert(stato == FERMO && hardware::levels[3] == LOW);
    Serial.receive("a\n"); tick(7000);
    assert(stato == PRE_GONFIAGGIO && hardware::levels[3] == HIGH);
    assert(tempoRichiestoMs == 200 && !precedenteSequenzaValida);
    assert(ultimoPeriodoMs == 0 && frontiPeriodo == 0 && numeroImpulsi == 1);
    Serial.receive("a\n"); tick(7500); ready(7000); edge(10000);
    const Trace trace = observe(10000, 10370);
    assert(trace.starts == std::vector<uint64_t>({10000}));
    assert(trace.lengths == std::vector<uint64_t>({70}));
  }
}

static void millis_rollover_during_distributed_sequence() {
  const uint64_t wrap = UINT32_MAX;
  reset(wrap - 8000); ready(wrap - 8000);
  edge(wrap - 2500); tick(wrap - 2430); tick(wrap - 2130);
  tempoRichiestoMs = 320; edge(wrap - 500);
  assert(ultimoPeriodoMs == 2000 && tempoRichiestoMs == 300);
  const Trace trace = observe(wrap - 500, wrap + 1370);
  assert(trace.starts == std::vector<uint64_t>({wrap - 500, wrap, wrap + 500, wrap + 1000}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70, 70, 70}));
  assert(stato == ATTENDI_FRONTE);
}

static void encoder_counter_rollover() {
  reset(); ready(); encoderTotale = UINT32_MAX - 2; totaleLetto = encoderTotale;
  edge(3000); edge(3010); edge(3020); tick(3070); tick(3370); edge(4000);
  assert(frontiPeriodo == 3 && ultimoPeriodoMs == 1000 && tempoRichiestoMs == 220);
  const Trace trace = observe(4000, 5036);
  assert(trace.starts == std::vector<uint64_t>({4000, 4333, 4666}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70, 70}));
}

static void long_period_offsets_use_64bit_products() {
  first_sequence(); tempoRichiestoMs = 320;
  const uint64_t start = 3000003000ULL;
  Serial.output.clear(); edge(start);
  assert(ultimoPeriodoMs == 3000000000UL && tempoRichiestoMs == 300 && numeroImpulsi == 4);
  assert(Serial.output.size() == 63 && Serial.output.find("Dt:3000000000") != std::string::npos);
  tick(start + 70); assert(hardware::levels[3] == LOW);
  for (const uint64_t offset : {750000000ULL, 1500000000ULL, 2250000000ULL}) {
    tick(start + offset - 1); assert(hardware::levels[3] == LOW);
    tick(start + offset); assert(hardware::levels[3] == HIGH);
    tick(start + offset + 69); assert(hardware::levels[3] == HIGH);
    tick(start + offset + 70); assert(hardware::levels[3] == LOW);
  }
  tick(start + 2250000370ULL); assert(stato == ATTENDI_FRONTE);
}

static void congested_serial_does_not_delay_the_sequence() {
  first_sequence(); tempoRichiestoMs = 320;
  Serial.output.clear(); Serial.txBlocked = true; edge(5000);
  const Trace trace = observe(5000, 6870, {}, true);
  assert(trace.starts == std::vector<uint64_t>({5000, 5500, 6000, 6500}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70, 70, 70}));
  assert(stato == ATTENDI_FRONTE && Serial.output.empty());
}

static void serial_logs_boot_and_every_physical_pulse_once() {
  reset(); assert(Serial.output == "Avvio freno\n");
  tick(999); tick(1000); tick(1001); ready();
  assert(Serial.output == "Avvio freno\n");
  Serial.output.clear(); edge(3000); tick(3001); tick(3070); edge(3100); tick(3370);
  assert(Serial.output == "Imp: 70 Corr: +0 Dt:         0 Fr:         0 Req:200 N:  1/  1\n");
  edge(4301); tick(4371); tick(4951); tick(5021);
  Serial.receive("s"); tick(5300);
  Serial.receive("a"); tick(5301);
  tick(7801); tick(8101); edge(8200);
  assert(Serial.output == "Imp: 70 Corr: +0 Dt:         0 Fr:         0 Req:200 N:  1/  1\n"
                          "Imp: 70 Corr:-20 Dt:      1301 Fr:         2 Req:180 N:  1/  2\n"
                          "Imp: 70 Corr: +0 Dt:      1301 Fr:         2 Req:180 N:  2/  2\n"
                          "Imp: 70 Corr: +0 Dt:         0 Fr:         0 Req:200 N:  1/  1\n");
}

static void serial_logs_need_space_for_the_whole_line() {
  reset(); ready(); Serial.output.clear();
  const std::string expected = "Imp: 70 Corr: +0 Dt:         0 Fr:         0 Req:200 N:  1/  1\n";
  assert(expected.size() == 63);
  Serial.txSpace = int(expected.size()) - 1; edge(3000);
  assert(Serial.output.empty() && hardware::levels[3] == HIGH);
  tick(3070); assert(Serial.output.empty() && hardware::levels[3] == LOW);
  reset(); ready(); Serial.output.clear();
  Serial.txSpace = int(expected.size()); edge(3000);
  assert(Serial.output == expected);
}

int main() {
  startup_and_inputs_without_pullup();
  debug_mirrors_raw_edges_and_holdoff_is_fixed();
  encoder_holdoff_micros_rollover();
  once_runs_only_on_state_entry();
  initial_wait_is_fixed_and_consumes_deadline_edge();
  example_has_four_70ms_pulses_500ms_apart();
  edges_do_not_restart_sequence_and_remain_in_next_sample();
  correction_uses_same_sequence_time_and_counter_window();
  fractional_spacing_does_not_accumulate_rounding();
  delayed_loop_measures_actual_sequence_start();
  late_loop_does_not_merge_or_shorten_pulses();
  maximum_requested_time_keeps_physical_pulses_fixed();
  minimum_request_still_generates_one_fixed_pulse();
  stop_and_restart_cancel_on_pulse_and_gap();
  millis_rollover_during_distributed_sequence();
  encoder_counter_rollover();
  long_period_offsets_use_64bit_products();
  congested_serial_does_not_delay_the_sequence();
  serial_logs_boot_and_every_physical_pulse_once();
  serial_logs_need_space_for_the_whole_line();
  std::cout << "20 gruppi di test PASS (simulazione, non validazione del prototipo)\n";
}
