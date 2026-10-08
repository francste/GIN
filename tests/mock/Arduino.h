#pragma once
#include <stdint.h>
#include <stddef.h>
#include <deque>
#include <sstream>
#include <string>
#include <stdexcept>

#define F(text) text
constexpr uint8_t A0 = 14;
constexpr int LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2, CHANGE = 3;

namespace hardware {
  extern uint64_t timeUs;
  extern int levels[24], modes[24], writes[24];
  extern void (*interrupt)();
  extern int interruptPin, interruptMode;
}
inline uint32_t micros() { return uint32_t(hardware::timeUs); }
inline uint32_t millis() { return uint32_t(hardware::timeUs / 1000); }
inline void digitalWrite(uint8_t pin, int level) {
  hardware::levels[pin] = level;
  ++hardware::writes[pin];
}
inline void pinMode(uint8_t pin, int mode) { hardware::modes[pin] = mode; }
inline int digitalPinToInterrupt(uint8_t pin) { return pin; }
inline void attachInterrupt(int pin, void (*fn)(), int mode) {
  hardware::interruptPin = pin;
  hardware::interruptMode = mode;
  hardware::interrupt = fn;
}

class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t) = 0;
  size_t write(const char *text) {
    size_t count = 0;
    while (*text) count += write(uint8_t(*text++));
    return count;
  }
  template <typename T> size_t print(const T &value) {
    std::ostringstream out;
    out << value;
    return write(out.str().c_str());
  }
  size_t print(uint8_t value) { return print(unsigned(value)); }
  size_t print(float value, int decimals) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(decimals);
    out << value;
    return write(out.str().c_str());
  }
  template <typename T> size_t println(const T &value) {
    return print(value) + write(uint8_t('\n'));
  }
  size_t println(float value, int decimals) {
    return print(value, decimals) + write(uint8_t('\n'));
  }
};

class MockSerial : public Print {
 public:
  using Print::write;
  std::deque<char> input;
  std::string output;
  bool txBlocked = false;
  void begin(unsigned long) {}
  int available() { return int(input.size()); }
  int read() { char c = input.front(); input.pop_front(); return c; }
  int availableForWrite() { return txBlocked ? 0 : 64; }
  size_t write(uint8_t value) override {
    if (txBlocked) throw std::runtime_error("scrittura UART bloccante durante il controllo");
    output.push_back(char(value));
    return 1;
  }
  void receive(const std::string &line) {
    for (char c : line) input.push_back(c);
  }
};
extern MockSerial Serial;
