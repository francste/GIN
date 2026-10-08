#pragma once

#include <stdint.h>

// Logica indipendente dall'hardware, con tempi senza attese bloccanti.
struct BrakeConfig {
  uint32_t targetEdgeMs = 600;       // Un fronte CHANGE ogni 600 ms in media.
  uint32_t quietMs = 1800;           // Arresto PRESUNTO dopo questo silenzio.
  uint32_t responseWaitMs = 800;     // Attesa DOPO lo spegnimento della pompa.
  uint32_t rateWindowMs = 6000;      // Recupero se il moto non si arresta mai.
  uint16_t minPulseMs = 50;
  uint16_t maxPulseMs = 200;
  uint16_t maxSessionOnMs = 3000;    // Tempo pompa cumulativo per singolo test.
  uint8_t maxPulsesPerBurst = 4;
  float proportionalMs = 100.0f;
  float adaptationMs = 5.0f;        // Variazione massima per aggiornamento.
  float smoothing = 0.25f;
  float deadband = 0.10f;
};

struct EncoderReading {
  uint32_t total;
  uint32_t lastEdgeMs;
  uint32_t meanPeriodUs;             // Media degli ultimi DUE intervalli.
};

class BrakeController {
 public:
  enum Event : uint8_t {
    NONE = 0, PUMP_ON = 1, PUMP_OFF = 2, CYCLE = 4, RATE = 8, FAULT = 16
  };
  enum Fault : uint8_t { OK = 0, TOO_MANY_PULSES = 1, ON_TIME_LIMIT = 2 };

  BrakeConfig config;
  bool armed = false;
  bool pumping = false;
  bool moving = false;
  Fault fault = OK;
  float biasMs = 25.0f;             // Può diventare negativo: permette di
                                    // saltare impulsi quando la media è bassa.
  float filteredError = 0.0f;
  float requestedMs = 0.0f;
  uint16_t pulseMs = 0;
  uint16_t sessionOnMs = 0;
  uint8_t burstPulses = 0;
  uint32_t burstEdges = 0;
  uint32_t cycleEdges = 0;
  uint32_t cycleMs = 0;
  uint32_t minPeriodUs = 0;
  uint32_t sampleEdges = 0;
  uint32_t sampleMs = 0;

  void arm(uint32_t now, uint32_t total) {
    stop(now);
    armed = true;
    fault = OK;
    sessionOnMs = 0;
    burstPulses = 0;
    burstEdges = 0;
    moving = false;
    filteredError = 0.0f;
    haveError = false;
    lastTotal = total;
    cycleStartMs = now;
    sampleStartMs = now;
    sampleStartTotal = total;
    // Il riarmo non cancella l'attesa di una pompa appena spenta.
  }

  void stop(uint32_t now) {
    armed = false;
    moving = false;
    if (pumping) {
      pumping = false;
      lastPumpStopMs = now;
    }
  }

  bool manualPulse(uint32_t now, uint16_t duration) {
    if (armed || fault != OK || duration < config.minPulseMs ||
        duration > config.maxPulseMs || !available(now) ||
        uint32_t(sessionOnMs) + duration > config.maxSessionOnMs) return false;
    requestedMs = duration;
    beginPulse(now, duration);
    return true;
  }

  uint8_t update(uint32_t now, const EncoderReading &encoder) {
    uint8_t events = NONE;
    // La scadenza agisce anche per gli impulsi manuali a controllo disarmato.
    if (pumping && elapsed(now, pumpStartMs) >= pulseMs) {
      pumping = false;
      lastPumpStopMs = now;
      events |= PUMP_OFF;
    }
    const uint32_t added = encoder.total - lastTotal;
    lastTotal = encoder.total;
    if (!armed) return events;

    if (added != 0) {
      if (!moving) {
        moving = true;
        burstEdges = 0;
        burstPulses = 0;
        minPeriodUs = 0;
      }
      burstEdges += added;          // Include fronti durante pompa E attesa.
      if (burstEdges >= 3 && encoder.meanPeriodUs != 0 &&
          (minPeriodUs == 0 || encoder.meanPeriodUs < minPeriodUs)) {
        minPeriodUs = encoder.meanPeriodUs;
      }
    }

    if (moving && elapsed(now, encoder.lastEdgeMs) >= config.quietMs) {
      cycleEdges = burstEdges;
      cycleMs = elapsed(now, cycleStartMs);
      adapt(now, encoder.total);
      cycleStartMs = now;
      moving = false;
      events |= CYCLE;
    } else if (moving && elapsed(now, sampleStartMs) >= config.rateWindowMs) {
      // Stesso adattamento, su intervalli disgiunti: nessun doppio conteggio
      // quando in seguito viene riconosciuto un arresto.
      if (adapt(now, encoder.total)) events |= RATE;
    }

    // Con CHANGE servono tre fronti per mediare un intervallo alto e uno basso.
    // Il primo fronte da solo non misura la velocità.
    if (!moving || added == 0 || burstEdges < 3 || !available(now)) return events;
    const float targetUs = float(config.targetEdgeMs) * 1000.0f;
    const float periodUs = float(encoder.meanPeriodUs);
    if (periodUs <= 0.0f || periodUs >= targetUs) return events;

    const float fastError = (targetUs - periodUs) / targetUs;
    requestedMs = clamp(biasMs + config.proportionalMs * fastError,
                        0.0f, float(config.maxPulseMs));
    // Non arrotondare una richiesta corta a 50 ms e non accodarla.
    if (requestedMs < config.minPulseMs) return events;

    if (burstPulses >= config.maxPulsesPerBurst) {
      fail(now, TOO_MANY_PULSES);
      return events | FAULT;
    }
    const uint16_t duration = uint16_t(requestedMs);
    if (uint32_t(sessionOnMs) + duration > config.maxSessionOnMs) {
      fail(now, ON_TIME_LIMIT);
      return events | FAULT;
    }
    beginPulse(now, duration);
    ++burstPulses;
    return events | PUMP_ON;
  }

 private:
  uint32_t lastTotal = 0;
  uint32_t cycleStartMs = 0;
  uint32_t sampleStartMs = 0;
  uint32_t sampleStartTotal = 0;
  uint32_t pumpStartMs = 0;
  uint32_t lastPumpStopMs = 0;
  bool havePumped = false;
  bool haveError = false;

  static uint32_t elapsed(uint32_t now, uint32_t since) { return now - since; }
  static float clamp(float value, float lower, float upper) {
    return value < lower ? lower : (value > upper ? upper : value);
  }
  bool available(uint32_t now) const {
    return !pumping && (!havePumped || elapsed(now, lastPumpStopMs) >= config.responseWaitMs);
  }
  void beginPulse(uint32_t now, uint16_t duration) {
    pulseMs = duration;
    pumpStartMs = now;
    pumping = true;
    havePumped = true;
    sessionOnMs += duration;
  }
  void fail(uint32_t now, Fault reason) {
    stop(now);
    fault = reason;
  }
  bool adapt(uint32_t now, uint32_t total) {
    const uint32_t duration = elapsed(now, sampleStartMs);
    if (duration < 2 * config.targetEdgeMs) return false;
    sampleMs = duration;
    sampleEdges = total - sampleStartTotal;
    const float error = float(sampleEdges) * config.targetEdgeMs / duration - 1.0f;
    if (!haveError) {
      filteredError = error;
      haveError = true;
    } else {
      filteredError += config.smoothing * (error - filteredError);
    }
    if (filteredError > config.deadband || filteredError < -config.deadband) {
      biasMs = clamp(biasMs + config.adaptationMs * clamp(filteredError, -1.0f, 1.0f),
                     -config.proportionalMs, float(config.maxPulseMs));
    }
    sampleStartMs = now;
    sampleStartTotal = total;
    return true;
  }
};
