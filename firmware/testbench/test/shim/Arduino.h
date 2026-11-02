// Stand-in for the Arduino core so src/main.cpp builds and runs on a laptop
// (pio test -e native). It only covers what main.cpp uses. Tests drive it
// through the sim:: variables: set the clock and inputs, read pins and output.
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <string>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LED_BUILTIN 13

static const uint8_t A0 = 14, A1 = 15, A2 = 16, A3 = 17, A4 = 18, A5 = 19;

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

namespace sim {
const int NPINS = 32;
inline unsigned long now_ms = 0;
inline bool tick_per_call = false;  // millis() advances 1 ms on every call
inline int pin_mode[NPINS];
inline int digital_out[NPINS];
inline int pwm_out[NPINS];
inline int digital_in[NPINS];  // what digitalRead returns
inline int analog_in[NPINS];   // what analogRead returns
inline std::string rx;         // bytes waiting for the firmware to read
inline std::string tx;         // everything the firmware printed

inline void reset_io() {
  for (int i = 0; i < NPINS; i++) {
    pin_mode[i] = INPUT;
    digital_out[i] = LOW;
    pwm_out[i] = 0;
    digital_in[i] = HIGH;  // pull-ups: buttons not pressed
    analog_in[i] = 0;
  }
  analog_in[A3] = 512;  // joystick centered
  rx.clear();
  tx.clear();
  tick_per_call = false;
}
}  // namespace sim

inline unsigned long millis() { return sim::tick_per_call ? sim::now_ms++ : sim::now_ms; }
inline void pinMode(uint8_t pin, uint8_t mode) { sim::pin_mode[pin] = mode; }
inline void digitalWrite(uint8_t pin, uint8_t value) { sim::digital_out[pin] = value; }
inline int digitalRead(uint8_t pin) { return sim::digital_in[pin]; }
inline void analogWrite(uint8_t pin, int value) { sim::pwm_out[pin] = value; }
inline int analogRead(uint8_t pin) { return sim::analog_in[pin]; }

class SimSerial {
 public:
  void begin(unsigned long) {}
  int available() { return (int)sim::rx.size(); }
  int read() {
    if (sim::rx.empty()) return -1;
    int c = (unsigned char)sim::rx[0];
    sim::rx.erase(0, 1);
    return c;
  }

  void print(const char *s) { sim::tx += s; }
  void print(const __FlashStringHelper *s) { sim::tx += reinterpret_cast<const char *>(s); }
  void print(char c) { sim::tx += c; }
  void print(int v) { sim::tx += std::to_string(v); }
  void print(unsigned int v) { sim::tx += std::to_string(v); }
  void print(long v) { sim::tx += std::to_string(v); }
  void print(unsigned long v) { sim::tx += std::to_string(v); }
  void print(double v, int digits = 2) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.*f", digits, v);
    sim::tx += buf;
  }

  template <class T>
  void println(T v) {
    print(v);
    println();
  }
  void println(double v, int digits) {
    print(v, digits);
    println();
  }
  void println() { sim::tx += "\r\n"; }
};

inline SimSerial Serial;
