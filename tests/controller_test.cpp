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
  precedenteSequenzaMs = totalePrimaMotorOn = 0;
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

static void first_episode(unsigned extraEdges = 0) {
  reset(); ready(); edge(3000);
  std::vector<uint64_t> edges;
  for (unsigned i = 0; i < extraEdges; ++i) edges.push_back(3002 + i * 2);
  observe(3000, 3000 + (extraEdges + 1) * 600, edges);
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
  assert(frontiMotorOn == extraEdges + 1 && correzioneTempoMs == 0);
}

// Il primo ciclo ha eseguito mantenimento; il secondo, con Corr=1400, non ne esegue.
// Il ciclo successivo puo' quindi ritardare il primo B se Corr rimane positiva.
static void previous_episode_without_maintenance() {
  first_episode(); edge(5000); tick(5150); tick(5350);
  assert(stato == ATTENDI_FRONTE && frontiMotorOn == 1 && numeroBloccaggi == 1);
  assert(correzioneTempoMs == 1400 && numeroMantenimenti == 0 && indiceMantenimento == 0);
}

// Dt=1200, n precedente=2, Corr=0; il nuovo MOTOR_ON conta tre fronti.
static void example_start() {
  first_episode(1); Serial.output.clear(); edge(4200); edge(4210); edge(4220);
  assert(ultimoPeriodoMs == 1200 && frontiPrecedenti == 2 && correzioneTempoMs == 0);
  assert(frontiMotorOn == 3 && numeroBloccaggi == 1);
}

static void example_maintenance() {
  example_start(); tick(4350); tick(4549); assert(stato == MOTOR_ON);
  tick(4550);
  assert(stato == MANTENIMENTO && hardware::levels[3] == HIGH);
  assert(durataMotorOnMs == 350 && durataMantenimentoMs == 1450 && numeroMantenimenti == 3);
}

static void startup_pins_and_defaults() {
  reset();
  assert(IMPULSO_BLOCCAGGIO_MS == 150 && IMPULSO_MANTENIMENTO_MS == 100);
  assert(INTERVALLO_MANTENIMENTO_MS == 600 && MS_PER_FRONTE == 600);
  assert(MOTOR_ON_TIMEOUT_MS == 200 && PRE_GONFIAGGIO_MS == 1500);
  assert(correzioneTempoMs == 0 && frontiMotorOn == 0 && durataMantenimentoMs == 0);
  assert(fronteAvvioBloccaggio == 1);
  assert(hardware::levels[11] == HIGH && hardware::levels[6] == HIGH);
  assert(hardware::levels[4] == LOW && hardware::levels[3] == HIGH);
  assert(hardware::modes[A0] == INPUT && !hardware::pullups[A0]);
  assert(hardware::modes[12] == OUTPUT && hardware::levels[12] == LOW);
  assert(PORTD.DIRCLR == PIN6_bm && PORTD.PIN6CTRL == 0);
  assert(PORTA.DIRCLR == PIN6_bm && PORTA.PIN6CTRL == 0);
  assert(hardware::interruptPin == 14 && hardware::interruptMode == CHANGE);
  assert(Serial.output.find("Avvio freno - ritardo solo senza mantenimenti precedenti\n") == 0);
  assert(Serial.output.find("\nDt : delta t tra ingressi MOTOR_ON successivi\n") != std::string::npos);
  assert(Serial.output.find("\nCorr : correzione accumulata, sempre con segno + o -\n") != std::string::npos);
  const char *fields[] = {"Np", "Av", "B", "Imp", "Fr", "On", "Tm", "M", "M:k/N", "t", "d", "s", "a"};
  for (const char *field : fields) {
    const std::string start = "\n" + std::string(field) + " : ";
    assert(occurrences(Serial.output, start) == 1);
  }
}

static void pregonfiaggio_goes_directly_to_fresh_edge_wait() {
  reset(); edge(1000); edge(1498); edge(1500);
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
  assert(!precedenteSequenzaValida && encoderTotale == 3);
  tick(1501); assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
  edge(1502);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  assert(inizioSequenzaMs == 1502 && totalePrimaMotorOn == 3 && frontiMotorOn == 1);
  assert(ultimoPeriodoMs == 0 && frontiPrecedenti == 0 && correzioneTempoMs == 0);
}

static void isr_only_counts_and_main_loop_starts_full_pulse() {
  reset(); ready(); const int writesBefore = hardware::writes[3];
  edge(3000, false); edge(3010, false); edge(3020, false);
  assert(encoderTotale == 3 && hardware::levels[3] == LOW && hardware::writes[3] == writesBefore);
  assert(stato == ATTENDI_FRONTE && !precedenteSequenzaValida);
  tick(3250);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && inizioSequenzaMs == 3250);
  assert(totalePrimaMotorOn == 0 && frontiMotorOn == 3 && numeroBloccaggi == 1);
  tick(3399); assert(hardware::levels[3] == HIGH);
  tick(3400); assert(hardware::levels[3] == LOW && tempoBloccaggioMs == 150);
  tick(3599); assert(stato == MOTOR_ON);
  tick(3600); assert(stato == MANTENIMENTO && durataMantenimentoMs == 1450);
}

static void once_and_always_gpio_writes() {
  reset(); const int pregWrites = hardware::writes[3];
  tick(1000); tick(1499); assert(hardware::writes[3] == pregWrites);
  ready(); const int offWrites = hardware::writes[3];
  tick(1800); tick(2000); assert(hardware::writes[3] == offWrites);
  edge(3000); const int blockWrites = hardware::writes[3];
  tick(3001); tick(3149); assert(hardware::writes[3] == blockWrites);
  tick(3150); const int waitWrites = hardware::writes[3];
  tick(3200); tick(3349); assert(hardware::writes[3] == waitWrites);
  tick(3350); assert(hardware::writes[3] == waitWrites + 1);
  tick(3351); tick(3449); assert(hardware::writes[3] == waitWrites + 1);
  tick(3450); tick(3600); const int endWrites = hardware::writes[3];
  tick(9000); assert(hardware::writes[3] == endWrites && stato == ATTENDI_FRONTE);
  Serial.receive("s"); tick(9010); const int stopWrites = hardware::writes[3];
  Serial.receive("s"); tick(9020); assert(hardware::writes[3] == stopWrites);
}

static void first_pulse_timeout_and_log_order() {
  reset(); ready(); Serial.output.clear(); edge(3000);
  assert(Serial.output == "B:1 Imp:150 Fr:    1 t:         0 d:         0\n"
                          "Dt:         0 Np:    0 Corr:+0 Av:1\n");
  tick(3149); assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  tick(3150); assert(stato == MOTOR_ON && hardware::levels[3] == LOW && ultimoSpegnimentoMs == 3150);
  tick(3349); assert(occurrences(Serial.output, "Tm:") == 0);
  tick(3350);
  assert(Serial.output.find("Fr:    1 On:   350 Tm:   250 M:1\n") != std::string::npos);
  assert(Serial.output.find("M:1/1 t:       350 d:       350\n") != std::string::npos);
  tick(3450); tick(3600); tick(9000);
  assert(occurrences(Serial.output, "Corr:") == 1 && occurrences(Serial.output, "Tm:") == 1);
}

static void encoder_during_high_is_counted_without_queue() {
  reset(); ready(); edge(3000);
  const Trace trace = observe(3000, 6600, {3005, 3010, 3080, 3148, 3150});
  assert(trace.starts == std::vector<uint64_t>({3000, 3350, 3950, 4550, 5150, 5750, 6350}));
  assert(trace.lengths == std::vector<uint64_t>({150, 100, 100, 100, 100, 100, 100}));
  assert(numeroBloccaggi == 1 && tempoBloccaggioMs == 150 && frontiMotorOn == 6);
  assert(stato == ATTENDI_FRONTE && durataMantenimentoMs == 3250);
}

static void two_to_four_blocking_pulses_without_recalculating_correction() {
  for (uint32_t count = 2; count <= 4; ++count) {
    reset(); ready(); Serial.output.clear(); edge(3000);
    std::vector<uint64_t> edges, starts;
    for (uint64_t i = 0; i < count; ++i) {
      starts.push_back(3000 + i * 180);
      if (i > 0) edges.push_back(starts.back());
      edges.push_back(starts.back() + 20);
    }
    const uint64_t deadline = starts.back() + 150 + 200;
    const Trace trace = observe(3000, deadline - 1, edges);
    assert(trace.starts == starts && trace.lengths == std::vector<uint64_t>(count, 150));
    assert(stato == MOTOR_ON && numeroBloccaggi == count && tempoBloccaggioMs == count * 150);
    assert(frontiMotorOn == count * 2 && inizioSequenzaMs == 3000);
    assert(correzioneTempoMs == 0 && occurrences(Serial.output, "Corr:") == 1);
    tick(deadline);
    assert(stato == MANTENIMENTO && durataMotorOnMs == deadline - 3000);
    assert(durataMantenimentoMs == count * 1200 - (deadline - 3000));
    assert(occurrences(Serial.output, "Corr:") == 1);
  }
}

static void timeout_retriggered_at_end_not_start() {
  reset(); ready(); edge(3000); tick(3150); edge(3180);
  tick(3330); assert(ultimoSpegnimentoMs == 3330 && numeroBloccaggi == 2);
  tick(3350); tick(3380); tick(3529); assert(stato == MOTOR_ON);
  tick(3530);
  assert(stato == MANTENIMENTO && tempoBloccaggioMs == 300 && frontiMotorOn == 2);
  assert(durataMotorOnMs == 530 && durataMantenimentoMs == 670);
}

static void fresh_edge_at_timeout_has_priority() {
  reset(); ready(); edge(3000); tick(3150); Serial.output.clear(); edge(3350);
  assert(stato == MOTOR_ON && numeroBloccaggi == 2 && hardware::levels[3] == HIGH);
  assert(Serial.output.find("Tm:") == std::string::npos && Serial.output.find("Corr:") == std::string::npos);
  tick(3500); tick(3699); assert(stato == MOTOR_ON);
  tick(3700); assert(stato == MANTENIMENTO && tempoBloccaggioMs == 300 && durataMotorOnMs == 700);
}

static void multiple_edges_in_off_gap_start_one_pulse_in_main() {
  reset(); ready(); edge(3000); tick(3150); const int writesBefore = hardware::writes[3];
  edge(3170, false); edge(3172, false); edge(3180, false);
  assert(hardware::levels[3] == LOW && hardware::writes[3] == writesBefore);
  tick(3190); assert(hardware::levels[3] == HIGH && numeroBloccaggi == 2 && frontiMotorOn == 4);
  tick(3340); tick(3540);
  assert(stato == MANTENIMENTO && numeroBloccaggi == 2 && durataMantenimentoMs == 1860);
}

static void edge_at_pulse_end_is_consumed_then_next_fresh_edge_restarts() {
  reset(); ready(); edge(3000); edge(3150);
  assert(stato == MOTOR_ON && hardware::levels[3] == LOW && numeroBloccaggi == 1 && frontiMotorOn == 2);
  tick(3151); assert(hardware::levels[3] == LOW);
  edge(3152); assert(hardware::levels[3] == HIGH && numeroBloccaggi == 2 && frontiMotorOn == 3);
  tick(3302); tick(3502); assert(stato == MANTENIMENTO && tempoBloccaggioMs == 300);
}

static void previous_and_current_counts_are_separate_and_no_edge_is_lost() {
  example_start(); tick(4350); edge(4380); edge(4390); tick(4530);
  assert(stato == MOTOR_ON && numeroBloccaggi == 2 && frontiMotorOn == 5);
  assert(ultimoPeriodoMs == 1200 && frontiPrecedenti == 2 && correzioneTempoMs == 0);
  tick(4730); assert(stato == MANTENIMENTO && durataMantenimentoMs == 2470);
  edge(4900);
  assert(stato == MOTOR_ON && ultimoPeriodoMs == 700 && frontiPrecedenti == 5);
  assert(correzioneTempoMs == -2300 && frontiMotorOn == 1);
  assert(encoderTotale == 8); // 2 + 5 + 1: ogni fronte assegnato a un solo ciclo.
}

static void regular_maintenance_uses_fixed_100ms_pulses_and_600ms_spacing() {
  reset(); ready(); edge(3000);
  const Trace trace = observe(3000, 4800, {3002, 3004});
  assert(trace.starts == std::vector<uint64_t>({3000, 3350, 3950, 4550}));
  assert(trace.lengths == std::vector<uint64_t>({150, 100, 100, 100}));
  assert(frontiMotorOn == 3 && durataMotorOnMs == 350 && durataMantenimentoMs == 1450);
  assert(numeroMantenimenti == 3 && indiceMantenimento == 3 && stato == ATTENDI_FRONTE);
  assert(Serial.output.find("M:3/3 t:      1550 d:       600") != std::string::npos);
}

static void elapsed_motor_on_time_includes_gaps_and_final_timeout() {
  reset(); ready(); edge(3000);
  const Trace trace = observe(3000, 4800, {3170, 3340});
  assert(trace.starts == std::vector<uint64_t>({3000, 3170, 3340, 3690, 4290}));
  assert(trace.lengths == std::vector<uint64_t>({150, 150, 150, 100, 100}));
  assert(tempoBloccaggioMs == 450 && durataMotorOnMs == 690 && frontiMotorOn == 3);
  assert(durataMantenimentoMs == 1110 && numeroMantenimenti == 2);
  assert(stato == ATTENDI_FRONTE); // 690 + 1110 = 3 * 600.
}

static void positive_correction_shortens_and_negative_correction_extends_window() {
  first_episode(); edge(4000); edge(4010); edge(4020); tick(4150); tick(4350);
  assert(correzioneTempoMs == 400 && frontiPrecedenti == 1 && frontiMotorOn == 3);
  assert(durataMantenimentoMs == 1050); // 1800 - 350 - 400: nessuna attesa dopo mantenimento.
  first_episode(1); edge(4300); edge(4310); edge(4320); tick(4450); tick(4650);
  assert(correzioneTempoMs == 100 && durataMantenimentoMs == 1350);
  edge(4800); assert(correzioneTempoMs == -1200 && frontiPrecedenti == 3 && frontiMotorOn == 1);
  tick(4950); tick(5150);
  assert(durataMantenimentoMs == 1450); // 600 - 350 - (-1200).
}

static void correction_accumulates_once_and_zero_error_preserves_it() {
  first_episode(1);
  edge(4700); edge(4702); tick(4850); observe(5050, 5400);
  assert(correzioneTempoMs == 500 && frontiMotorOn == 2 && durataMantenimentoMs == 350);
  edge(5700); edge(5702); tick(5850); observe(6050, 6600);
  assert(correzioneTempoMs == 300 && durataMantenimentoMs == 550);
  Serial.output.clear(); edge(6900);
  assert(ultimoPeriodoMs == 1200 && frontiPrecedenti == 2 && correzioneTempoMs == 300);
  edge(6910); tick(7050); edge(7080); tick(7230); tick(7430);
  assert(correzioneTempoMs == 300 && occurrences(Serial.output, "Corr:") == 1);
  assert(frontiMotorOn == 3 && durataMantenimentoMs == 970);
}

static void short_windows_only_allow_full_pulses() {
  const uint32_t windows[] = {0, 1, 99, 100, 101, 699, 700, 701};
  const size_t counts[] = {0, 0, 0, 1, 1, 1, 2, 2};
  for (size_t i = 0; i < 8; ++i) {
    reset(); ready();
    correzioneTempoMs = 250 - int64_t(windows[i]);
    edge(3000);
    const Trace trace = observe(3000, 3350 + windows[i] + 1000);
    assert(trace.starts.size() == 1 + counts[i]);
    std::vector<uint64_t> durations(counts[i] + 1, 100); durations.front() = 150;
    assert(trace.lengths == durations && numeroMantenimenti == counts[i]);
    assert(durataMantenimentoMs == windows[i] && stato == ATTENDI_FRONTE);
  }
}

static void maintenance_gap_is_interrupted_without_minimum_restart_distance() {
  example_maintenance(); tick(4650); const int writesBefore = hardware::writes[3];
  edge(4800, false);
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW && hardware::writes[3] == writesBefore);
  tick(4800);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && inizioSequenzaMs == 4800);
  assert(ultimoPeriodoMs == 600 && frontiPrecedenti == 3 && frontiMotorOn == 1);
  assert(correzioneTempoMs == -1200 && numeroMantenimenti == 0);
  tick(4950); tick(5150);
  assert(durataMantenimentoMs == 1450 && numeroMantenimenti == 3);
}

static void maintenance_on_is_interrupted_without_low_glitch() {
  example_maintenance(); const int writesBefore = hardware::writes[3]; edge(4580);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && hardware::writes[3] == writesBefore + 1);
  assert(inizioImpulsoMs == 4580 && frontiPrecedenti == 3 && frontiMotorOn == 1);
  assert(ultimoPeriodoMs == 380 && correzioneTempoMs == -1420);
  tick(4729); assert(hardware::levels[3] == HIGH);
  tick(4730); assert(hardware::levels[3] == LOW && tempoBloccaggioMs == 150);
  tick(4930); assert(stato == MANTENIMENTO && durataMantenimentoMs == 1670);
}

static void encoder_has_priority_at_all_maintenance_deadlines() {
  for (const uint64_t when : {4650, 5150, 6000}) {
    example_maintenance(); observe(4551, when - 1); Serial.output.clear(); edge(when);
    assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && inizioSequenzaMs == when);
    assert(numeroBloccaggi == 1 && frontiPrecedenti == 3 && frontiMotorOn == 1);
    assert(Serial.output.find("M:") == std::string::npos);
    tick(when + 149); assert(hardware::levels[3] == HIGH);
    tick(when + 150); assert(hardware::levels[3] == LOW);
  }
}

static void window_expiration_enters_wait_and_wait_never_restarts_by_itself() {
  example_maintenance(); observe(4551, 5999);
  assert(stato == MANTENIMENTO && hardware::levels[3] == LOW && indiceMantenimento == 3);
  tick(6000); assert(stato == ATTENDI_FRONTE && !mantenimentoAcceso);
  const int writesBefore = hardware::writes[3]; tick(20000);
  assert(stato == ATTENDI_FRONTE && hardware::writes[3] == writesBefore);
  edge(21000, false); assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW);
  tick(21000); assert(stato == MOTOR_ON && hardware::levels[3] == HIGH);
  assert(ultimoPeriodoMs == 16800 && frontiPrecedenti == 3 && frontiMotorOn == 1);
  assert(correzioneTempoMs == 15000 && fronteAvvioBloccaggio == 1 && numeroBloccaggi == 1);
}

static void negative_window_is_zero_without_unsigned_underflow() {
  first_episode(); edge(5000);
  assert(fronteAvvioBloccaggio == 1 && hardware::levels[3] == HIGH);
  tick(5150); tick(5350);
  assert(correzioneTempoMs == 1400 && durataMotorOnMs == 350);
  assert(durataMantenimentoMs == 0 && numeroMantenimenti == 0 && stato == ATTENDI_FRONTE);
  assert(hardware::levels[3] == LOW);
  assert(limitaDurataMs(-1) == 0 && limitaDurataMs(0) == 0);
}

static void delayed_loop_uses_actual_elapsed_time_and_never_catches_up() {
  reset(); ready(); edge(3000); tick(3500);
  assert(tempoBloccaggioMs == 500 && hardware::levels[3] == LOW);
  tick(3699); assert(stato == MOTOR_ON); tick(3700);
  assert(durataMotorOnMs == 700 && durataMantenimentoMs == 0 && stato == ATTENDI_FRONTE);
  reset(); ready(); edge(3000);
  std::vector<uint64_t> edges; for (uint64_t i = 1; i < 10; ++i) edges.push_back(3000 + i * 2);
  observe(3000, 3450, edges); tick(5000);
  assert(hardware::levels[3] == HIGH && inizioImpulsoMs == 5000);
  tick(5100); tick(5150); tick(5599); assert(hardware::levels[3] == LOW);
  tick(5600); assert(hardware::levels[3] == HIGH && indiceMantenimento == 3);
  assert(Serial.output.find("M:3/10 t:      2600 d:       600") != std::string::npos);
}

static void window_deadline_stops_even_a_pulse_that_loop_failed_to_end() {
  reset(); ready(); edge(3000); tick(3150); tick(3350);
  assert(hardware::levels[3] == HIGH); tick(3600);
  assert(stato == ATTENDI_FRONTE && hardware::levels[3] == LOW && !mantenimentoAcceso);
  const int writesBefore = hardware::writes[3]; tick(9000);
  assert(hardware::writes[3] == writesBefore);
}

static void millis_and_encoder_counter_rollover() {
  const uint64_t wrap = uint64_t(UINT32_MAX) + 1;
  reset(wrap - 8000); ready(wrap - 8000); edge(wrap - 150);
  const Trace trace = observe(wrap - 150, wrap + 1650, {wrap - 140, wrap - 130});
  assert(trace.starts == std::vector<uint64_t>({wrap - 150, wrap + 200, wrap + 800, wrap + 1400}));
  assert(trace.lengths == std::vector<uint64_t>({150, 100, 100, 100}));
  assert(stato == ATTENDI_FRONTE && durataMotorOnMs == 350 && durataMantenimentoMs == 1450);
  edge(wrap + 1650); assert(ultimoPeriodoMs == 1800 && frontiPrecedenti == 3 && correzioneTempoMs == 0);
  reset(); ready(); encoderTotale = UINT32_MAX - 2; ultimoTotaleLetto = encoderTotale;
  edge(3000); observe(3000, 4800, {3002, 3004});
  assert(encoderTotale == 0 && frontiMotorOn == 3);
  edge(4800); assert(frontiPrecedenti == 3 && frontiMotorOn == 1 && correzioneTempoMs == 0);
}

static void large_counts_use_64bit_products_and_numerical_limits() {
  reset(); ready(); edge(3000);
  encoderTotale = totalePrimaMotorOn + UINT32_MAX;
  tick(3150); tick(3350);
  assert(frontiMotorOn == UINT32_MAX && durataMantenimentoMs == UINT32_MAX);
  assert(numeroMantenimenti == 7158279UL);
  tick(3450); edge(4000);
  assert(frontiPrecedenti == UINT32_MAX && frontiMotorOn == 1);
  assert(correzioneTempoMs == -int64_t(UINT32_MAX));
  tick(4150); tick(4350); assert(durataMantenimentoMs == UINT32_MAX);
  assert(limitaDurataMs(int64_t(UINT32_MAX) + 1) == UINT32_MAX);
  first_episode(); correzioneTempoMs = UINT32_MAX - 50;
  edge(4000); assert(correzioneTempoMs == int64_t(UINT32_MAX));
}

static void stale_loop_clock_does_not_shorten_first_pulse_and_on_sum_is_diagnostic() {
  reset(); ready(); edge(3000, false); aggiornaFreno(2999);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && inizioSequenzaMs == 3000);
  tick(3149); assert(hardware::levels[3] == HIGH);
  tempoBloccaggioMs = UINT32_MAX - 20; tick(3150);
  assert(tempoBloccaggioMs == UINT32_MAX && hardware::levels[3] == LOW);
  tick(3350); assert(durataMantenimentoMs == 250); // Conta il tempo nello stato, non la somma HIGH.
}

static void stop_cancels_all_states_and_restart_resets_correction() {
  for (const uint64_t stopTime : {4250, 4400, 4580, 4800, 6100}) {
    example_maintenance();
    if (stopTime < 4550) example_start();
    else observe(4551, stopTime - 1);
    Serial.receive("s\n"); tick(stopTime);
    assert(stato == FERMO && hardware::levels[3] == LOW);
    edge(stopTime + 100); tick(stopTime + 200); assert(stato == FERMO && hardware::levels[3] == LOW);
    correzioneTempoMs = 1234; Serial.receive("a\n"); tick(7000);
    assert(stato == PRE_GONFIAGGIO && correzioneTempoMs == 0 && !precedenteSequenzaValida);
    ready(7000); edge(9000); tick(9150); tick(9350);
    assert(stato == MANTENIMENTO && frontiMotorOn == 1 && durataMantenimentoMs == 250);
    assert(ultimoPeriodoMs == 0 && frontiPrecedenti == 0);
  }
  example_maintenance(); edge(4800, false); Serial.receive("s"); tick(4800);
  assert(stato == FERMO && hardware::levels[3] == LOW && precedenteSequenzaMs == 4200);
}

static void serial_congestion_does_not_change_physical_spacing() {
  reset(); ready(); Serial.output.clear(); Serial.txBlocked = true; edge(3000);
  const Trace trace = observe(3000, 4800, {3002, 3004}, true);
  assert(trace.starts == std::vector<uint64_t>({3000, 3350, 3950, 4550}));
  assert(trace.lengths == std::vector<uint64_t>({150, 100, 100, 100}));
  assert(Serial.output.empty() && stato == ATTENDI_FRONTE);
  example_start(); tick(4350); Serial.output.clear(); Serial.txSpace = 0;
  tick(4550); tick(4650); assert(Serial.output.empty());
  Serial.txSpace = 63; tick(5150);
  assert(Serial.output.find("M:2/3 t:       950 d:       600\n") != std::string::npos);
  tick(5250); tick(5750); tick(5850);
  assert(Serial.output.find("M:3/3 t:      1550 d:       600\n") != std::string::npos);
}

static void log_requires_whole_rows_and_prints_signed_large_correction() {
  reset(); ready(); Serial.output.clear();
  const std::string first = "B:1 Imp:150 Fr:    1 t:         0 d:         0\n";
  Serial.txSpace = int(first.size()) - 1; edge(3000);
  assert(Serial.output.find("B:") == std::string::npos && hardware::levels[3] == HIGH);
  reset(); ready(); Serial.output.clear(); Serial.txSpace = int(first.size()); edge(3000);
  assert(Serial.output.find(first) == 0);
  Serial.output.clear(); Serial.txSpace = 63; correzioneTempoMs = UINT32_MAX; stampaIntervallo(); svuotaLog();
  assert(Serial.output.find("Corr:+4294967295") != std::string::npos);
  Serial.output.clear(); correzioneTempoMs = -int64_t(UINT32_MAX); stampaIntervallo(); svuotaLog();
  assert(Serial.output.find("Corr:-4294967295") != std::string::npos);
  Serial.output.clear(); inviaLog("questa riga non entra", 30, 20);
  assert(Serial.output.empty());
}

static void proportional_start_thresholds_and_reported_corrections() {
  const int64_t values[] = {-1000, -1, 0, 1, 599, 600, 601, 1200, 1400, 22675, 23336, 23789, UINT32_MAX};
  const uint32_t thresholds[] = {1, 1, 1, 2, 2, 2, 3, 3, 4, 39, 40, 41, 7158280};
  for (size_t i = 0; i < 13; ++i) assert(calcolaFronteAvvio(values[i]) == thresholds[i]);
  assert(MS_CORR_PER_FRONTE_ATTESO == 600);
}

static void second_or_third_front_starts_first_pulse_without_wait_timeout() {
  for (const int correction : {1, 600, 601, 1200}) {
    previous_episode_without_maintenance(); correzioneTempoMs = correction; edge(5600);
    const uint32_t target = correction <= 600 ? 2 : 3;
    assert(stato == MOTOR_ON && fronteAvvioBloccaggio == target && hardware::levels[3] == LOW);
    tick(10000); // Nessun timeout prima dell'effettivo primo B.
    assert(stato == MOTOR_ON && numeroBloccaggi == 0 && frontiMotorOn == 1);
    for (uint32_t n = 2; n <= target; ++n) {
      edge(10000 + n * 10);
      assert(frontiMotorOn == n && hardware::levels[3] == (n == target ? HIGH : LOW));
    }
    const uint64_t firstOn = 10000 + target * 10;
    assert(numeroBloccaggi == 1 && inizioBloccaggioMs == firstOn && inizioSequenzaMs == 5600);
    tick(firstOn + 149); assert(hardware::levels[3] == HIGH);
    tick(firstOn + 150); tick(firstOn + 349); assert(stato == MOTOR_ON);
    tick(firstOn + 350); assert(stato == ATTENDI_FRONTE && durataMotorOnMs == firstOn + 350 - 5600);
  }
}

static void pending_encoder_edges_before_main_satisfy_threshold_once() {
  previous_episode_without_maintenance(); correzioneTempoMs = 500;
  edge(5600, false); edge(5610, false);
  assert(hardware::levels[3] == LOW && encoderTotale == 4);
  tick(5700); // Dt=700: Corr cresce da 500 a 600 prima della scelta di Av.
  assert(stato == MOTOR_ON && fronteAvvioBloccaggio == 2 && frontiMotorOn == 2);
  assert(correzioneTempoMs == 600 && numeroBloccaggi == 1);
  assert(hardware::levels[3] == HIGH && inizioBloccaggioMs == 5700);
  tick(5850); assert(hardware::levels[3] == LOW);
}

static void gate_counts_waiting_edges_then_normal_blocking_retriggers() {
  previous_episode_without_maintenance(); correzioneTempoMs = 601; edge(5600);
  const Trace trace = observe(5600, 6221, {5650, 5720, 5730, 5872});
  assert(trace.starts == std::vector<uint64_t>({5720, 5872}));
  assert(trace.lengths == std::vector<uint64_t>({150, 150}));
  assert(frontiMotorOn == 5 && numeroBloccaggi == 2 && fronteAvvioBloccaggio == 3);
  assert(correzioneTempoMs == 601 && ultimoSpegnimentoMs == 6022);
  assert(Serial.output.find("B:1 Imp:150 Fr:    3 t:         0 d:         0") != std::string::npos);
  tick(6222);
  assert(durataMotorOnMs == 622 && durataMantenimentoMs == 1777 && tempoBloccaggioMs == 300);
  assert(Serial.output.find("M:1/3 t:       502 d:       350") != std::string::npos);
}

static void positive_corr_after_active_maintenance_starts_without_low_glitch() {
  example_maintenance(); correzioneTempoMs = 2000;
  const int writesBefore = hardware::writes[3]; edge(4580);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && numeroBloccaggi == 1);
  assert(hardware::writes[3] == writesBefore + 1); // Nessuno spegnimento intermedio.
  assert(correzioneTempoMs == 580 && frontiPrecedenti == 3 && frontiMotorOn == 1);
  assert(inizioSequenzaMs == 4580 && fronteAvvioBloccaggio == 1 && inizioBloccaggioMs == 4580);
  edge(4590); edge(4600); tick(4730); tick(4930);
  assert(frontiMotorOn == 3 && durataMotorOnMs == 350 && durataMantenimentoMs == 870);
}

static void completed_maintenance_prevents_delay_even_with_large_positive_corr() {
  for (const unsigned extraEdges : {0, 2}) {
    for (const int correction : {1, 600, 601, 22675}) {
      first_episode(extraEdges);
      assert(indiceMantenimento == extraEdges + 1);
      const uint64_t nextStart = 3000 + (extraEdges + 1) * 600;
      correzioneTempoMs = correction; Serial.output.clear(); edge(nextStart);
      assert(correzioneTempoMs == correction && fronteAvvioBloccaggio == 1);
      assert(numeroBloccaggi == 1 && hardware::levels[3] == HIGH);
      assert(inizioSequenzaMs == nextStart && inizioBloccaggioMs == nextStart);
      tick(nextStart + 149); assert(hardware::levels[3] == HIGH);
      tick(nextStart + 150); assert(hardware::levels[3] == LOW);
      assert(Serial.output.find(" Av:1\n") != std::string::npos);
    }
  }
}

static void no_maintenance_uses_updated_corr_and_actual_pulse_count() {
  for (const int correctionBefore : {-200, -100, -99}) {
    previous_episode_without_maintenance(); correzioneTempoMs = correctionBefore; edge(5700);
    // Dt=700, Np=1: il nuovo errore e' +100 ms. Decide la Corr aggiornata.
    assert(correzioneTempoMs == correctionBefore + 100);
    const bool delayed = correctionBefore + 100 > 0;
    assert(fronteAvvioBloccaggio == (delayed ? 2U : 1U));
    assert(hardware::levels[3] == (delayed ? LOW : HIGH));
  }
  previous_episode_without_maintenance(); correzioneTempoMs = 10; edge(5352);
  assert(correzioneTempoMs == -238 && fronteAvvioBloccaggio == 1 && hardware::levels[3] == HIGH);

  previous_episode_without_maintenance(); correzioneTempoMs = 600;
  // Finestra nominale con impulsi previsti ma nessuno avviato, per ritardo del loop.
  numeroMantenimenti = 3; assert(indiceMantenimento == 0); edge(5600);
  assert(correzioneTempoMs == 600 && fronteAvvioBloccaggio == 2 && hardware::levels[3] == LOW);
}

static void maintenance_disables_delay_again_after_gated_cycle() {
  previous_episode_without_maintenance(); correzioneTempoMs = 600; edge(5600);
  assert(fronteAvvioBloccaggio == 2 && hardware::levels[3] == LOW);
  edge(5700); tick(5850); tick(6050);
  assert(durataMotorOnMs == 450 && durataMantenimentoMs == 150 && indiceMantenimento == 1);
  tick(6150); tick(6200); assert(stato == ATTENDI_FRONTE);
  edge(6800);
  assert(correzioneTempoMs == 600 && fronteAvvioBloccaggio == 1);
  assert(numeroBloccaggi == 1 && hardware::levels[3] == HIGH && inizioBloccaggioMs == 6800);
  tick(6950); edge(6970);
  assert(numeroBloccaggi == 2 && frontiMotorOn == 2 && correzioneTempoMs == 600);
}

static void negative_and_zero_corr_start_immediately_and_logs_keep_sign() {
  for (const int correction : {-1000, -1, 0}) {
    reset(); ready(); Serial.output.clear(); correzioneTempoMs = correction; edge(3000);
    assert(hardware::levels[3] == HIGH && fronteAvvioBloccaggio == 1 && numeroBloccaggi == 1);
    const std::string expected = "Corr:" + std::string(correction < 0 ? "" : "+") + std::to_string(correction);
    assert(Serial.output.find(expected + " Av:1") != std::string::npos);
  }
  previous_episode_without_maintenance(); Serial.output.clear(); correzioneTempoMs = 600; edge(5600);
  assert(Serial.output.find("Corr:+600 Av:2") != std::string::npos);
  assert(Serial.output.find("B:") == std::string::npos);
}

static void stop_during_gate_then_restart_resets_wait() {
  previous_episode_without_maintenance(); correzioneTempoMs = 22675; edge(5600);
  assert(fronteAvvioBloccaggio == 39 && hardware::levels[3] == LOW);
  edge(5700, false); Serial.receive("s"); tick(5700);
  assert(stato == FERMO && hardware::levels[3] == LOW && numeroBloccaggi == 0);
  edge(5800); Serial.receive("a"); tick(6500);
  assert(stato == PRE_GONFIAGGIO && fronteAvvioBloccaggio == 1 && correzioneTempoMs == 0);
  ready(6500); edge(8500);
  assert(stato == MOTOR_ON && hardware::levels[3] == HIGH && frontiMotorOn == 1);
}

static void gated_start_and_counter_across_millis_rollover() {
  const uint64_t wrap = uint64_t(UINT32_MAX) + 1;
  previous_episode_without_maintenance();
  // Compensa il grande Dt del salto temporale: la Corr aggiornata all'ingresso vale 601.
  correzioneTempoMs = 601 - (int64_t(wrap - 250 - 5000) - MS_PER_FRONTE);
  encoderTotale = UINT32_MAX - 1; ultimoTotaleLetto = encoderTotale;
  edge(wrap - 250);
  const Trace trace = observe(wrap - 250, wrap + 1549, {wrap - 125, wrap - 100, wrap - 50});
  assert(trace.starts == std::vector<uint64_t>({wrap - 100, wrap + 250, wrap + 850}));
  assert(trace.lengths == std::vector<uint64_t>({150, 100, 100}));
  assert(stato == ATTENDI_FRONTE && frontiMotorOn == 4 && encoderTotale == 2);
  assert(durataMotorOnMs == 500 && durataMantenimentoMs == 1299);
  edge(wrap + 1549);
  assert(ultimoPeriodoMs == 1799 && frontiPrecedenti == 4 && correzioneTempoMs == 0);
  assert(fronteAvvioBloccaggio == 1 && hardware::levels[3] == HIGH);
}

static void full_uart_does_not_delay_threshold_pulse_or_timeout() {
  previous_episode_without_maintenance(); Serial.output.clear(); Serial.txBlocked = true;
  correzioneTempoMs = 601; edge(5600);
  const Trace trace = observe(5600, 7400, {5700, 5800, 5810}, true);
  assert(trace.starts == std::vector<uint64_t>({5800, 6150, 6750}));
  assert(trace.lengths == std::vector<uint64_t>({150, 100, 100}));
  assert(frontiMotorOn == 4 && durataMotorOnMs == 550 && durataMantenimentoMs == 1249);
  assert(stato == ATTENDI_FRONTE && Serial.output.empty());
}

static void negative_corr_survives_uart_filling_after_first_block_log() {
  reset(); ready(); Serial.output.clear(); Serial.consumeTxSpace = true;
  Serial.txSpace = 63; correzioneTempoMs = -1; edge(3000);
  assert(hardware::levels[3] == HIGH && Serial.output.find("B:1") == 0);
  assert(Serial.output.find("Corr:") == std::string::npos && righeLogInAttesa == 1);
  Serial.txSpace = 63; tick(3001);
  assert(Serial.output.find("Corr:-1 Av:1") != std::string::npos && righeLogInAttesa == 0);
  assert(hardware::levels[3] == HIGH && numeroBloccaggi == 1);
}

static void full_log_queue_never_blocks_motor_or_overwrites_rows() {
  previous_episode_without_maintenance(); Serial.output.clear(); Serial.txBlocked = true;
  correzioneTempoMs = 600; edge(5600);
  for (unsigned i = 0; i < 10; ++i) stampaIntervallo();
  assert(righeLogInAttesa == 4 && Serial.output.empty());
  edge(5700); assert(hardware::levels[3] == HIGH && numeroBloccaggi == 1);
  tick(5850); assert(hardware::levels[3] == LOW && righeLogInAttesa == 4);
  Serial.txBlocked = false; Serial.txSpace = 63; Serial.consumeTxSpace = true;
  for (uint64_t ms = 5851; ms < 5855; ++ms) { Serial.txSpace = 63; tick(ms); }
  assert(righeLogInAttesa == 0 && occurrences(Serial.output, "Corr:+600 Av:2") == 4);
}

static void low_load_release_model_recovers_positive_corr() {
  // A freno libero arriva un fronte ogni 100 ms; HIGH blocca subito e
  // il freno si sblocca 3000 ms dopo LOW. Il modello usa solo le uscite fisiche.
  reset(); ready(); edge(3000);
  std::vector<uint64_t> starts{3000};
  uint64_t releaseAt = 0, nextFreeEdge = 3100;
  bool moving = true, wasOn = false;
  int64_t maxCorr = 0;
  uint32_t completedEdges = 0;
  for (uint64_t ms = 3000; starts.size() < 301 && ms < 2000000; ++ms) {
    if (releaseAt == ms) {
      completedEdges = encoderTotale; // Esclude il primo fronte del prossimo ciclo.
      edge(ms); starts.push_back(ms); moving = true; nextFreeEdge = ms + 100; releaseAt = 0;
    } else if (moving && nextFreeEdge == ms) {
      edge(ms); nextFreeEdge += 100;
    } else tick(ms);
    const bool on = hardware::levels[3] == HIGH;
    if (on) { moving = false; releaseAt = 0; }
    else if (wasOn) releaseAt = ms + 3000;
    wasOn = on;
    if (correzioneTempoMs > maxCorr) maxCorr = correzioneTempoMs;
  }
  assert(starts.size() == 301 && completedEdges > 1500);
  const double meanMs = double(starts.back() - starts.front()) / completedEdges;
  assert(meanMs > 597 && meanMs < 603 && maxCorr <= 6000);
  std::cout << "Modello basso carico, rilascio 3000 ms: media " << meanMs
            << " ms/fronte, Corr massima " << maxCorr << " ms\n";
}

static void fixed_release_delay_model_keeps_mean_cadence_near_target() {
  // Modello semplificato: quattro fronti per movimento, arresto col primo B,
  // sblocco 800 ms dopo ogni spegnimento. Misura le uscite per retriggerare il modello.
  reset(); ready(); edge(3000);
  std::vector<uint64_t> starts{3000};
  uint64_t releaseAt = 0, burstAt = 3010;
  unsigned burstRemaining = 3;
  bool wasOn = true;
  for (uint64_t ms = 3001; starts.size() < 101 && ms < 300000; ++ms) {
    if (releaseAt == ms) {
      edge(ms); starts.push_back(ms); burstAt = ms + 10; burstRemaining = 3;
      assert(frontiPrecedenti == 4 && numeroBloccaggi == 1 && fronteAvvioBloccaggio == 1);
    } else if (burstRemaining > 0 && burstAt == ms) {
      edge(ms); --burstRemaining; burstAt += 10;
    } else tick(ms);
    const bool on = hardware::levels[3] == HIGH;
    if (on) releaseAt = 0;
    else if (wasOn) releaseAt = ms + 800;
    wasOn = on;
  }
  assert(starts.size() == 101);
  const double meanMsPerEdge = double(starts.back() - starts.front()) / (100 * 4);
  assert(meanMsPerEdge > 597 && meanMsPerEdge < 603);
  assert(correzioneTempoMs >= 0 && correzioneTempoMs < 1000);
  std::cout << "Modello ritardo 800 ms: media " << meanMsPerEdge << " ms/fronte\n";
}

int main() {
  startup_pins_and_defaults();
  pregonfiaggio_goes_directly_to_fresh_edge_wait();
  debug_and_fixed_holdoff();
  micros_rollover_holdoff();
  isr_only_counts_and_main_loop_starts_full_pulse();
  once_and_always_gpio_writes();
  first_pulse_timeout_and_log_order();
  encoder_during_high_is_counted_without_queue();
  two_to_four_blocking_pulses_without_recalculating_correction();
  timeout_retriggered_at_end_not_start();
  fresh_edge_at_timeout_has_priority();
  multiple_edges_in_off_gap_start_one_pulse_in_main();
  edge_at_pulse_end_is_consumed_then_next_fresh_edge_restarts();
  previous_and_current_counts_are_separate_and_no_edge_is_lost();
  regular_maintenance_uses_fixed_100ms_pulses_and_600ms_spacing();
  elapsed_motor_on_time_includes_gaps_and_final_timeout();
  positive_correction_shortens_and_negative_correction_extends_window();
  correction_accumulates_once_and_zero_error_preserves_it();
  short_windows_only_allow_full_pulses();
  maintenance_gap_is_interrupted_without_minimum_restart_distance();
  maintenance_on_is_interrupted_without_low_glitch();
  encoder_has_priority_at_all_maintenance_deadlines();
  window_expiration_enters_wait_and_wait_never_restarts_by_itself();
  negative_window_is_zero_without_unsigned_underflow();
  delayed_loop_uses_actual_elapsed_time_and_never_catches_up();
  window_deadline_stops_even_a_pulse_that_loop_failed_to_end();
  millis_and_encoder_counter_rollover();
  large_counts_use_64bit_products_and_numerical_limits();
  stale_loop_clock_does_not_shorten_first_pulse_and_on_sum_is_diagnostic();
  stop_cancels_all_states_and_restart_resets_correction();
  serial_congestion_does_not_change_physical_spacing();
  log_requires_whole_rows_and_prints_signed_large_correction();
  proportional_start_thresholds_and_reported_corrections();
  second_or_third_front_starts_first_pulse_without_wait_timeout();
  pending_encoder_edges_before_main_satisfy_threshold_once();
  gate_counts_waiting_edges_then_normal_blocking_retriggers();
  positive_corr_after_active_maintenance_starts_without_low_glitch();
  completed_maintenance_prevents_delay_even_with_large_positive_corr();
  no_maintenance_uses_updated_corr_and_actual_pulse_count();
  maintenance_disables_delay_again_after_gated_cycle();
  negative_and_zero_corr_start_immediately_and_logs_keep_sign();
  stop_during_gate_then_restart_resets_wait();
  gated_start_and_counter_across_millis_rollover();
  full_uart_does_not_delay_threshold_pulse_or_timeout();
  negative_corr_survives_uart_filling_after_first_block_log();
  full_log_queue_never_blocks_motor_or_overwrites_rows();
  low_load_release_model_recovers_positive_corr();
  fixed_release_delay_model_keeps_mean_cadence_near_target();
  std::cout << "48 gruppi di test PASS (simulazione, non validazione del prototipo)\n";
}
