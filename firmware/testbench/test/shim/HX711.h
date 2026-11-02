// Stand-in for the bogde/HX711 library (see Arduino.h in this folder).
// Tests set sim::hx711_present and sim::hx711_grams (the load on the cell).
#pragma once

#include <stdint.h>

namespace sim {
inline bool hx711_present = true;
inline double hx711_grams = 0.0;
inline double hx711_counts_per_gram = 455.0;  // the "true" calibration
}  // namespace sim

class HX711 {
 public:
  void begin(uint8_t, uint8_t) {}
  bool wait_ready_timeout(unsigned long) { return sim::hx711_present; }
  bool is_ready() { return sim::hx711_present; }
  void set_scale(float s) { scale_ = s; }
  void tare(uint8_t = 10) { offset_ = raw(); }
  double get_value(uint8_t = 1) { return raw() - offset_; }
  float get_units(uint8_t = 1) { return (float)(get_value() / scale_); }

 private:
  double raw() { return 8000.0 + sim::hx711_grams * sim::hx711_counts_per_gram; }
  float scale_ = 1.0f;
  double offset_ = 0.0;
};
