// Stand-in for the Arduino EEPROM library (see Arduino.h in this folder).
#pragma once

#include <stdint.h>
#include <string.h>

class SimEEPROM {
 public:
  SimEEPROM() { erase(); }
  void erase() { memset(mem, 0xFF, sizeof(mem)); }  // blank EEPROM reads 0xFF

  template <class T>
  T &get(int addr, T &t) {
    memcpy(&t, mem + addr, sizeof(T));
    return t;
  }
  template <class T>
  const T &put(int addr, const T &t) {
    memcpy(mem + addr, &t, sizeof(T));
    return t;
  }

  uint8_t mem[1024];
};

inline SimEEPROM EEPROM;
