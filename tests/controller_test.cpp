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
  encoderPronto = bloccaggioRichiesto = false;
  bloccaggioMs = bloccaggioFronti = 0;
  ultimoFronteValidoUs = 0; fronteValidoRicevuto = false;
  precedenteSequenzaMs = totaleAllaSequenza = 0;
  at(startMs); setup();
}

static void ready(uint64_t startMs = 0) {
  tick(startMs + 1499);
  assert(stato == PRE_GONFIAGGIO && hardware::levels[3] == HIGH);
  tick(startMs + 1500);
  assert(stato == ATTENDI_ARRESTO && hardware::levels[3] == LOW);
  tick(startMs + 1799); assert(stato == ATTENDI_ARRESTO);
  tick(startMs + 1800); assert(stato == ATTENDI_FRONTE);
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

static void first_sequence(unsigned frontiExtra = 0) {
  reset(); ready(); edge(3000);
  for (unsigned i = 0; i < frontiExtra; ++i) edge(3002 + i * 2);
  tick(3149); assert(hardware::levels[3] == HIGH);
  tick(3150); assert(hardware::levels[3] == LOW && stato == MANTENIMENTO);
  assert(encoderPronto);
  tick(3299); assert(hardware::levels[3] == LOW);
  tick(3300); assert(hardware::levels[3] == HIGH);
  tick(3369); assert(hardware::levels[3] == HIGH);
  tick(3370); tick(3450); assert(stato == MANTENIMENTO && hardware::levels[3] == LOW);
}

// Dt=2000, n=2: tempo medio 1000 ms, Corr=-40. Richiesta da 340 a 300 ms.
static void example_sequence() {
  first_sequence(1); tempoRichiestoMs = 340; edge(5000);
  assert(tempoRichiestoMs == 300 && ultimoPeriodoMs == 2000 && frontiPeriodo == 2);
}

static void startup_and_inputs_without_pullup() {
  reset();
  assert(IMPULSO_BLOCCAGGIO_MS == 150 && IMPULSO_MANTENIMENTO_MS == 70);
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
  assert(tempoRichiestoMs == 220 && frontiPeriodo == 0 && ultimoPeriodoMs == 0);
  assert(numeroImpulsi == 2 && totaleAllaSequenza == 2 && periodoDistribuzioneMs == 600);
  const Trace trace = observe(3000, 3370);
  assert(trace.starts == std::vector<uint64_t>({3000, 3300}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70}));
  tick(10000);
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW && encoderPronto);
}

static void first_sequence_with_high_request_keeps_maintenance_without_dt() {
  reset(); ready(); tempoRichiestoMs = 400; edge(3000);
  assert(ultimoPeriodoMs == 0 && frontiPeriodo == 0 && tempoRichiestoMs == 400);
  assert(numeroImpulsi == 4 && periodoDistribuzioneMs == 604);
  const Trace trace = observe(3000, 3523);
  assert(trace.starts == std::vector<uint64_t>({3000, 3151, 3302, 3453}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70, 70, 70}));
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW && encoderPronto);
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
  tick(1000); tick(1200); tick(1499);
  assert(hardware::writes[3] == bootWrites && inizioStatoMs == 0);
  ready(); edge(3000);
  const int onWrites = hardware::writes[3];
  tick(3001); edge(3010); tick(3149);
  assert(hardware::writes[3] == onWrites && precedenteSequenzaMs == 3000);
  tick(3150);
  assert(hardware::writes[3] == onWrites + 1 && hardware::levels[3] == LOW);
  tick(3200); tick(3299);
  assert(hardware::writes[3] == onWrites + 1 && inizioStatoMs == 3150);
  tick(3300); tick(3310);
  assert(hardware::writes[3] == onWrites + 2 && inizioStatoMs == 3300);
  tick(3370); tick(3449);
  assert(hardware::writes[3] == onWrites + 3 && hardware::levels[3] == LOW);
  Serial.receive("s"); tick(3500);
  const int stopWrites = hardware::writes[3];
  Serial.receive("s"); tick(3600);
  assert(hardware::writes[3] == stopWrites);
}

static void initial_wait_is_fixed_and_consumes_deadline_edge() {
  reset(); tick(1500); edge(1600); edge(1798);
  assert(stato == ATTENDI_ARRESTO && hardware::levels[3] == LOW && inizioStatoMs == 1500);
  edge(1800);
  assert(stato == ATTENDI_FRONTE && !precedenteSequenzaValida);
  tick(1801); assert(hardware::levels[3] == LOW);
  edge(1802);
  assert(stato == MOTOR_ON && precedenteSequenzaMs == 1802 && totaleAllaSequenza == 4);
  tick(1951); assert(hardware::levels[3] == HIGH);
  tick(1952); assert(hardware::levels[3] == LOW);
}

static void example_subtracts_blocking_before_maintenance() {
  example_sequence();
  const Trace trace = observe(5000, 6703);
  assert(trace.starts == std::vector<uint64_t>({5000, 5666, 6333}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70, 70}));
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW && encoderPronto);
  tick(10000); assert(hardware::levels[3] == LOW); // Nessun riavvio automatico.
}

static void edge_in_gap_cancels_maintenance_and_blocks_in_isr() {
  example_sequence(); tick(5150);
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW && encoderPronto);
  Serial.output.clear(); edge(5200, false);
  // Il motore parte prima che il loop cambi stato, ricalcoli o stampi.
  assert(hardware::levels[3] == HIGH && stato == MANTENIMENTO);
  assert(bloccaggioRichiesto && !encoderPronto && Serial.output.empty());
  tick(5200);
  assert(stato == MOTOR_ON && precedenteSequenzaMs == 5200);
  assert(ultimoPeriodoMs == 200 && frontiPeriodo == 1 && tempoRichiestoMs == 340);
  assert(numeroImpulsi == 3 && periodoDistribuzioneMs == 453);
  assert(Serial.output == "Imp:150 Corr: +40 Dt:       200 Fr:    1 Req:340 1/3 t:0\n");
  tick(5349); assert(hardware::levels[3] == HIGH);
  tick(5350); assert(stato == MANTENIMENTO && hardware::levels[3] == LOW);
  const Trace trace = observe(5351, 6703);
  // La nuova sequenza ha i suoi mantenimenti; gli avvii precedenti 5666 e 6333 sono cancellati.
  assert(trace.starts == std::vector<uint64_t>({5351, 5502}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70}));
  assert(Serial.output == "Imp:150 Corr: +40 Dt:       200 Fr:    1 Req:340 1/3 t:0\n"
                          "2/3 t:       151 d:       151\n"
                          "3/3 t:       302 d:       151\n");
}

static void edge_during_on_maintenance_keeps_high_for_new_block() {
  example_sequence(); tick(5150); Serial.output.clear();
  const Trace trace = observe(5666, 6500, {5700});
  // 34 ms di mantenimento gia' trascorsi + 150 ms dal nuovo fronte, senza spegnimento.
  assert(trace.starts == std::vector<uint64_t>({5666, 5933, 6166}));
  assert(trace.lengths == std::vector<uint64_t>({184, 70, 70}));
  assert(precedenteSequenzaMs == 5700 && ultimoPeriodoMs == 700 && frontiPeriodo == 1);
  assert(tempoRichiestoMs == 290 && stato == MANTENIMENTO);
  assert(Serial.output == "2/3 t:       666 d:       666\n"
                          "Imp:150 Corr: -10 Dt:       700 Fr:    1 Req:290 1/3 t:0\n"
                          "2/3 t:       233 d:       233\n"
                          "3/3 t:       466 d:       233\n");
}

static void encoder_has_priority_at_maintenance_start_and_end() {
  example_sequence(); tick(5150); Serial.output.clear();
  edge(5666); // Istante programmato del secondo impulso della vecchia sequenza.
  assert(stato == MOTOR_ON && precedenteSequenzaMs == 5666);
  assert(Serial.output == "Imp:150 Corr:  -7 Dt:       666 Fr:    1 Req:293 1/3 t:0\n");
  tick(5815); assert(hardware::levels[3] == HIGH);
  tick(5816); assert(hardware::levels[3] == LOW);

  example_sequence(); tick(5150); tick(5666);
  const int writesBeforeEdge = hardware::writes[3];
  Serial.output.clear(); edge(5736); // Scadenza esatta dei 70 ms.
  assert(hardware::levels[3] == HIGH && hardware::writes[3] == writesBeforeEdge + 1);
  assert(stato == MOTOR_ON && precedenteSequenzaMs == 5736);
  tick(5885); assert(hardware::levels[3] == HIGH);
  tick(5886); assert(hardware::levels[3] == LOW);
}

static void no_timeout_after_block_or_final_maintenance() {
  reset(); ready(); edge(3000); tick(3150);
  edge(3152, false);
  assert(hardware::levels[3] == HIGH && bloccaggioRichiesto);
  tick(3152);
  assert(stato == MOTOR_ON && ultimoPeriodoMs == 152 && tempoRichiestoMs == 265);
  assert(numeroImpulsi == 2 && periodoDistribuzioneMs == 302);

  example_sequence(); observe(5000, 6403);
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW && encoderPronto);
  edge(6405, false);
  assert(hardware::levels[3] == HIGH && bloccaggioRichiesto);
  tick(6405);
  assert(stato == MOTOR_ON && precedenteSequenzaMs == 6405 && ultimoPeriodoMs == 1405);
}

static void correction_uses_same_sequence_time_and_counter_window() {
  first_sequence(); tempoRichiestoMs = 290; edge(3500); // T=500, n=1: +10, richiesta 300.
  const Trace trace = observe(3500, 4203);
  assert(trace.starts == std::vector<uint64_t>({3500, 3666, 3833}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70, 70}));
  assert(precedenteSequenzaMs == 3500 && tempoRichiestoMs == 300);
  edge(4204); // T=704, n=1: -10, richiesta 290.
  assert(ultimoPeriodoMs == 704 && frontiPeriodo == 1 && tempoRichiestoMs == 290);

  first_sequence(); edge(3600); // T=600, n=1: invariata, blocco + mantenimento.
  assert(ultimoPeriodoMs == 600 && tempoRichiestoMs == 220 && numeroImpulsi == 2);
  first_sequence(); edge(3601); // T=601, n=1: -0,1 ms arrotonda a zero.
  assert(ultimoPeriodoMs == 601 && tempoRichiestoMs == 220 && numeroImpulsi == 2);
}

static void proportional_correction_matches_measured_samples() {
  const uint32_t periods[] = {3340, 2867, 3738, 2865, 4285, 4553, 5059};
  const uint32_t edges[] = {8, 14, 4, 47, 30, 29, 5};
  const int corrections[] = {18, 40, -33, 54, 46, 44, -41};
  for (size_t i = 0; i < 7; ++i) {
    first_sequence(); tempoRichiestoMs = 300;
    encoderTotale = totaleAllaSequenza + edges[i] - 1;
    edge(3000 + periods[i]);
    assert(ultimoPeriodoMs == periods[i] && frontiPeriodo == edges[i]);
    assert(tempoRichiestoMs == 300 + corrections[i]);
  }
}

static void proportional_gain_uses_average_and_rounds_symmetrically() {
  assert(KP_PER_MILLE == 100);
  assert(correzioneProporzionaleMs(500, 1) == 10);
  assert(correzioneProporzionaleMs(1000, 2) == 10); // Stessa media: stessa correzione.
  assert(correzioneProporzionaleMs(300, 1) == 30);
  assert(correzioneProporzionaleMs(700, 1) == -10);
  assert(correzioneProporzionaleMs(900, 1) == -30);
  assert(correzioneProporzionaleMs(599, 1) == 0);
  assert(correzioneProporzionaleMs(601, 1) == 0);
  assert(correzioneProporzionaleMs(595, 1) == 1);
  assert(correzioneProporzionaleMs(605, 1) == -1);
  assert(correzioneProporzionaleMs(1190, 2) == 1);
  assert(correzioneProporzionaleMs(1210, 2) == -1);
  assert(correzioneProporzionaleMs(1200, 2) == 0);
  assert(correzioneProporzionaleMs(1000, 0) == 0);
}

static void proportional_limits_and_large_counters() {
  assert(correzioneProporzionaleMs(UINT32_MAX, UINT32_MAX) == 60);
  assert(correzioneProporzionaleMs(UINT32_MAX, 1) == -429496670LL);
  assert(limitaRichiestaMs(500) == 400 && limitaRichiestaMs(0) == 220);
  first_sequence(); tempoRichiestoMs = 400; Serial.output.clear(); edge(3500);
  assert(tempoRichiestoMs == 400 && Serial.output.find("Corr:  +0") != std::string::npos);
  first_sequence(); tempoRichiestoMs = 400; edge(3000ULL + UINT32_MAX);
  assert(tempoRichiestoMs == 220 && ultimoPeriodoMs == UINT32_MAX && frontiPeriodo == 1);
  assert(numeroImpulsi == 2);
}

static void fractional_spacing_does_not_accumulate_rounding() {
  first_sequence(3); tempoRichiestoMs = 390; edge(5003);
  assert(tempoRichiestoMs == 400 && numeroImpulsi == 4);
  const Trace trace = observe(5003, 6875);
  assert(trace.starts == std::vector<uint64_t>({5003, 5503, 6004, 6505}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70, 70, 70}));
  assert(stato == MANTENIMENTO);
}

static void delayed_loop_measures_actual_sequence_start() {
  reset(); ready();
  edge(3000, false);
  assert(hardware::levels[3] == HIGH && bloccaggioMs == 3000 && bloccaggioFronti == 1);
  edge(3010, false); edge(3020, false); tick(3050);
  assert(precedenteSequenzaMs == 3000 && totaleAllaSequenza == 1 && encoderTotale == 3);
  assert(bloccaggioMs == 3000 && bloccaggioFronti == 1); // Il primo campione resta congelato.
  tick(3149); assert(hardware::levels[3] == HIGH);
  tick(3150); assert(hardware::levels[3] == LOW && stato == MANTENIMENTO);
  Serial.output.clear(); edge(3601, false); tick(3602);
  assert(ultimoPeriodoMs == 601 && frontiPeriodo == 3 && tempoRichiestoMs == 260);
  assert(Serial.output == "Imp:150 Corr: +40 Dt:       601 Fr:    3 Req:260 1/2 t:0\n");
  tick(3750); assert(hardware::levels[3] == HIGH);
  tick(3751); assert(hardware::levels[3] == LOW);
}

static void encoder_arriving_after_loop_clock_does_not_expire_new_block() {
  reset(); ready();
  const uint32_t previousNow = 2999;
  edge(3000, false);
  aggiornaFreno(previousNow); // Equivale a un fronte arrivato tra millis() e la copia atomica.
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && inizioStatoMs == 3000);
  tick(3149); assert(hardware::levels[3] == HIGH);
  tick(3150); assert(hardware::levels[3] == LOW);
}

static void late_loop_does_not_merge_or_shorten_pulses() {
  example_sequence(); tick(7000); // Anche lo spegnimento viene osservato in ritardo.
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW);
  tick(7000); assert(hardware::levels[3] == LOW); // Almeno 1 ms spento.
  const Trace trace = observe(7001, 7442);
  assert(trace.starts == std::vector<uint64_t>({7001, 7072}));
  assert(trace.lengths == std::vector<uint64_t>({70, 70}));
  assert(precedenteSequenzaMs == 5000 && ultimoPeriodoMs == 2000);
  assert(stato == MANTENIMENTO);
  assert(Serial.output.find("2/3 t:      2001 d:      2001\n") != std::string::npos);
  assert(Serial.output.find("3/3 t:      2072 d:        71\n") != std::string::npos);
}

static void maximum_requested_time_keeps_physical_pulses_fixed() {
  reset(); ready();
  const uint16_t requests[] = {220, 230, 240, 250, 260, 270, 280, 290, 300,
                              310, 320, 330, 340, 350, 360, 370, 380, 390,
                              400, 400, 400, 400, 400, 400, 400};
  const size_t counts[] = {2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
                           3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
  for (uint64_t cycle = 0; cycle < 25; ++cycle) {
    const uint64_t start = 3000 + cycle * 2000;
    Serial.output.clear(); edge(start);
    assert(tempoRichiestoMs == requests[cycle]);
    if (cycle >= 19)
      assert(Serial.output == "Imp:150 Corr:  +0 Dt:      2000 Fr:    4 Req:400 1/4 t:0\n");
    const Trace trace = observe(start, start + 1999, {start + 20, start + 40, start + 60});
    std::vector<uint64_t> lengths(counts[cycle], 70); lengths.front() = 150;
    assert(trace.starts.size() == counts[cycle] && trace.lengths == lengths);
    assert(stato == MANTENIMENTO);
  }
}

static void minimum_request_still_generates_blocking_and_maintenance() {
  reset(); ready(); tempoRichiestoMs = 310;
  const uint16_t requests[] = {310, 270, 230, 220, 220, 220, 220, 220,
                              220, 220, 220, 220, 220, 220, 220};
  for (uint64_t cycle = 0; cycle < 15; ++cycle) {
    const uint64_t start = 3000 + cycle * 1000;
    Serial.output.clear(); edge(start);
    assert(tempoRichiestoMs == requests[cycle]);
    if (cycle == 3)
      assert(Serial.output == "Imp:150 Corr: -10 Dt:      1000 Fr:    1 Req:220 1/2 t:0\n");
    if (cycle >= 4)
      assert(Serial.output == "Imp:150 Corr:  +0 Dt:      1000 Fr:    1 Req:220 1/2 t:0\n");
    const Trace trace = observe(start, start + 999);
    assert(trace.starts.size() == (cycle == 0 ? 3 : 2));
    assert(trace.lengths == (cycle == 0 ? std::vector<uint64_t>({150, 70, 70})
                                      : std::vector<uint64_t>({150, 70})));
    assert(stato == MANTENIMENTO);
  }
}

static void stop_and_restart_cancel_on_pulse_and_gap() {
  for (const uint64_t stopTime : {5010, 5300, 5670}) {
    example_sequence();
    if (stopTime >= 5300) {
      tick(5150);
      assert(stato == MANTENIMENTO && hardware::levels[3] == LOW);
    }
    if (stopTime == 5670) { tick(5666); assert(hardware::levels[3] == HIGH); }
    Serial.receive("s\n"); tick(stopTime);
    assert(stato == FERMO && hardware::levels[3] == LOW);
    edge(5800); tick(6000); tick(6500);
    assert(stato == FERMO && hardware::levels[3] == LOW);
    Serial.receive("a\n"); tick(7000);
    assert(stato == PRE_GONFIAGGIO && hardware::levels[3] == HIGH);
    assert(tempoRichiestoMs == 220 && !precedenteSequenzaValida);
    assert(ultimoPeriodoMs == 0 && frontiPeriodo == 0 && numeroImpulsi == 2);
    Serial.receive("a\n"); tick(7500); ready(7000); edge(10000);
    const Trace trace = observe(10000, 10450);
    assert(trace.starts == std::vector<uint64_t>({10000, 10300}));
    assert(trace.lengths == std::vector<uint64_t>({150, 70}));
  }
}

static void stop_cancels_pending_isr_request_before_recalculation() {
  example_sequence(); tick(5150); Serial.output.clear();
  edge(5200, false);
  assert(hardware::levels[3] == HIGH && bloccaggioRichiesto);
  Serial.receive("s"); tick(5200);
  assert(stato == FERMO && hardware::levels[3] == LOW);
  assert(!encoderPronto && !bloccaggioRichiesto && Serial.output.empty());
  assert(precedenteSequenzaMs == 5000 && tempoRichiestoMs == 300);
  edge(5300, false); tick(6000);
  assert(hardware::levels[3] == LOW && stato == FERMO && Serial.output.empty());
}

static void millis_rollover_during_distributed_sequence() {
  const uint64_t wrap = UINT32_MAX;
  reset(wrap - 8000); ready(wrap - 8000);
  edge(wrap - 2500); edge(wrap - 2480); edge(wrap - 2460); edge(wrap - 2440);
  tick(wrap - 2350); tick(wrap - 2200); tick(wrap - 2130); tick(wrap - 2050);
  Serial.output.clear();
  tempoRichiestoMs = 390; edge(wrap - 500);
  assert(ultimoPeriodoMs == 2000 && tempoRichiestoMs == 400);
  const Trace trace = observe(wrap - 500, wrap + 1370);
  assert(trace.starts == std::vector<uint64_t>({wrap - 500, wrap, wrap + 500, wrap + 1000}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70, 70, 70}));
  assert(stato == MANTENIMENTO);
  assert(Serial.output.find("2/4 t:       500 d:       500\n") != std::string::npos);
  assert(Serial.output.find("3/4 t:      1000 d:       500\n") != std::string::npos);
  assert(Serial.output.find("4/4 t:      1500 d:       500\n") != std::string::npos);
}

static void encoder_counter_rollover() {
  reset(); ready(); encoderTotale = UINT32_MAX - 2;
  tempoRichiestoMs = 280;
  edge(3000); edge(3010); edge(3020); tick(3150); tick(3300); tick(3370); tick(3450); edge(4000);
  assert(frontiPeriodo == 3 && ultimoPeriodoMs == 1000 && tempoRichiestoMs == 307);
  const Trace trace = observe(4000, 5036);
  assert(trace.starts == std::vector<uint64_t>({4000, 4333, 4666}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70, 70}));
}

static void long_period_offsets_use_64bit_products() {
  first_sequence(); tempoRichiestoMs = 400;
  const uint64_t start = 3000003000ULL;
  encoderTotale += 4999999; // Dt=3e9, n=5e6: media 600 ms, correzione nulla.
  Serial.output.clear(); edge(start);
  assert(ultimoPeriodoMs == 3000000000UL && tempoRichiestoMs == 400 && numeroImpulsi == 4);
  assert(Serial.output.size() <= 63 && Serial.output.find("Dt:3000000000") != std::string::npos);
  tick(start + 149); assert(hardware::levels[3] == HIGH);
  tick(start + 150); assert(hardware::levels[3] == LOW);
  for (const uint64_t offset : {750000000ULL, 1500000000ULL, 2250000000ULL}) {
    tick(start + offset - 1); assert(hardware::levels[3] == LOW);
    tick(start + offset); assert(hardware::levels[3] == HIGH);
    tick(start + offset + 69); assert(hardware::levels[3] == HIGH);
    tick(start + offset + 70); assert(hardware::levels[3] == LOW);
  }
  tick(start + 2250000370ULL); assert(stato == MANTENIMENTO);
}

static void congested_serial_does_not_delay_the_sequence() {
  first_sequence(1); tempoRichiestoMs = 340;
  Serial.output.clear(); Serial.txBlocked = true; edge(5000);
  const Trace trace = observe(5000, 6703, {}, true);
  assert(trace.starts == std::vector<uint64_t>({5000, 5666, 6333}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70, 70}));
  assert(stato == MANTENIMENTO && Serial.output.empty());
}

static void serial_logs_boot_and_every_physical_pulse_once() {
  reset(); assert(Serial.output == "Avvio freno\n");
  tick(999); tick(1000); tick(1001); ready();
  assert(Serial.output == "Avvio freno\n");
  Serial.output.clear(); edge(3000); tick(3001); edge(3100); tick(3150);
  tick(3300); tick(3370); tick(3450);
  assert(Serial.output == "Imp:150 Corr:  +0 Dt:         0 Fr:    0 Req:220 1/2 t:0\n"
                          "2/2 t:       300 d:       300\n");
  tempoRichiestoMs = 305;
  edge(4301); tick(4451); tick(4734);
  assert(tempoRichiestoMs == 300 && ultimoPeriodoMs == 1301 && frontiPeriodo == 2);
  assert(precedenteSequenzaMs == 4301 && totaleAllaSequenza == 3);
  tick(4804); tick(5168);
  assert(tempoRichiestoMs == 300 && ultimoPeriodoMs == 1301 && frontiPeriodo == 2);
  assert(precedenteSequenzaMs == 4301 && totaleAllaSequenza == 3);
  tick(5238);
  Serial.receive("s"); tick(5300);
  Serial.receive("a"); tick(5301);
  tick(6801); tick(7101); edge(7200);
  assert(Serial.output == "Imp:150 Corr:  +0 Dt:         0 Fr:    0 Req:220 1/2 t:0\n"
                          "2/2 t:       300 d:       300\n"
                          "Imp:150 Corr:  -5 Dt:      1301 Fr:    2 Req:300 1/3 t:0\n"
                          "2/3 t:       433 d:       433\n"
                          "3/3 t:       867 d:       434\n"
                          "Imp:150 Corr:  +0 Dt:         0 Fr:    0 Req:220 1/2 t:0\n");
}

static void serial_logs_need_space_for_the_whole_line() {
  reset(); ready(); Serial.output.clear();
  const std::string expected = "Imp:150 Corr:  +0 Dt:         0 Fr:    0 Req:220 1/2 t:0\n";
  assert(expected.size() <= 63);
  Serial.txSpace = int(expected.size()) - 1; edge(3000);
  assert(Serial.output.empty() && hardware::levels[3] == HIGH);
  tick(3150); assert(Serial.output.empty() && hardware::levels[3] == LOW);
  reset(); ready(); Serial.output.clear();
  Serial.txSpace = int(expected.size()); edge(3000);
  assert(Serial.output == expected);

  example_sequence(); Serial.output.clear(); tick(5150);
  const std::string second = "2/3 t:       666 d:       666\n";
  const std::string third = "3/3 t:      1333 d:       667\n";
  Serial.txSpace = int(second.size()) - 1; tick(5666); // La riga non entra interamente.
  assert(Serial.output.empty() && hardware::levels[3] == HIGH);
  tick(5736); assert(hardware::levels[3] == LOW);
  Serial.txSpace = int(third.size()); tick(6333); // d resta fisico anche se il secondo log e' saltato.
  assert(Serial.output == third && hardware::levels[3] == HIGH);
  tick(6403); assert(hardware::levels[3] == LOW);
}

static void serial_timestamps_and_large_counters_fit_uart_buffer() {
  first_sequence(); tempoRichiestoMs = 340;
  encoderTotale = totaleAllaSequenza - 2; // Il prossimo fronte produce una differenza UINT32_MAX.
  const uint64_t start = 3000ULL + UINT32_MAX;
  Serial.output.clear(); Serial.txSpace = 63; edge(start);
  assert(Serial.output == "Imp:150 Corr: +60 Dt:4294967295 Fr:4294967295 Req:400 1/4 t:0\n");
  assert(Serial.output.size() == 62 && numeroImpulsi == 4);
  tick(start + 150); Serial.output.clear();
  tick(start + 1073741823ULL);
  assert(Serial.output == "2/4 t:1073741823 d:1073741823\n");
  tick(start + 1073741893ULL); Serial.output.clear();
  tick(start + 2147483647ULL);
  assert(Serial.output == "3/4 t:2147483647 d:1073741824\n");
}

static void maintenance_always_present_and_blocking_is_subtracted() {
  const uint16_t requests[] = {50, 149, 150, 200, 219, 220, 221, 289, 290, 300, 400};
  const size_t counts[] = {2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 4};
  const uint64_t totals[] = {220, 220, 220, 220, 220, 220, 220, 220, 290, 290, 360};
  for (size_t i = 0; i < 11; ++i) {
    reset(); ready(); edge(3000);
    // T=2400 e n=4: correzione nulla, per verificare la richiesta esatta.
    edge(3010); edge(3020); edge(3030); tick(3150); tick(3300); tick(3370);
    tempoRichiestoMs = requests[i]; edge(5400);
    assert(tempoRichiestoMs == (requests[i] < 220 ? 220 : requests[i]));
    const Trace trace = observe(5400, 8200);
    assert(trace.starts.size() == counts[i]);
    std::vector<uint64_t> lengths(counts[i], 70); lengths.front() = 150;
    assert(trace.lengths == lengths);
    uint64_t total = 0;
    for (const uint64_t length : trace.lengths) total += length;
    assert(total == totals[i]);
  }
}

static void short_period_leaves_room_for_long_blocking_pulse() {
  first_sequence(); tempoRichiestoMs = 390; edge(3500);
  assert(tempoRichiestoMs == 400 && ultimoPeriodoMs == 500);
  const Trace trace = observe(3500, 4203);
  // Con Dt=500 i quattro impulsi sono conservati e distanziati almeno di 151 ms.
  assert(periodoDistribuzioneMs == 604);
  assert(trace.starts == std::vector<uint64_t>({3500, 3651, 3802, 3953}));
  assert(trace.lengths == std::vector<uint64_t>({150, 70, 70, 70}));
  assert(stato == MANTENIMENTO);
}

int main() {
  startup_and_inputs_without_pullup();
  first_sequence_with_high_request_keeps_maintenance_without_dt();
  debug_mirrors_raw_edges_and_holdoff_is_fixed();
  encoder_holdoff_micros_rollover();
  once_runs_only_on_state_entry();
  initial_wait_is_fixed_and_consumes_deadline_edge();
  example_subtracts_blocking_before_maintenance();
  edge_in_gap_cancels_maintenance_and_blocks_in_isr();
  edge_during_on_maintenance_keeps_high_for_new_block();
  encoder_has_priority_at_maintenance_start_and_end();
  no_timeout_after_block_or_final_maintenance();
  correction_uses_same_sequence_time_and_counter_window();
  proportional_correction_matches_measured_samples();
  proportional_gain_uses_average_and_rounds_symmetrically();
  proportional_limits_and_large_counters();
  fractional_spacing_does_not_accumulate_rounding();
  delayed_loop_measures_actual_sequence_start();
  encoder_arriving_after_loop_clock_does_not_expire_new_block();
  late_loop_does_not_merge_or_shorten_pulses();
  maximum_requested_time_keeps_physical_pulses_fixed();
  minimum_request_still_generates_blocking_and_maintenance();
  stop_and_restart_cancel_on_pulse_and_gap();
  stop_cancels_pending_isr_request_before_recalculation();
  millis_rollover_during_distributed_sequence();
  encoder_counter_rollover();
  long_period_offsets_use_64bit_products();
  congested_serial_does_not_delay_the_sequence();
  serial_logs_boot_and_every_physical_pulse_once();
  serial_logs_need_space_for_the_whole_line();
  serial_timestamps_and_large_counters_fit_uart_buffer();
  maintenance_always_present_and_blocking_is_subtracted();
  short_period_leaves_room_for_long_blocking_pulse();
  std::cout << "32 gruppi di test PASS (simulazione, non validazione del prototipo)\n";
}
