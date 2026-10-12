#pragma once
#include <stdint.h>

class TwoWire {
public:
  void begin(int sda, int scl, uint32_t freq) {}
  void beginTransmission(uint8_t address) {}
  uint8_t write(uint8_t b) { return 1; }
  uint8_t endTransmission(bool sendStop) { return 0; }
  uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
    return quantity;
  }
  int read() { return 0; }
};

extern TwoWire Wire;
