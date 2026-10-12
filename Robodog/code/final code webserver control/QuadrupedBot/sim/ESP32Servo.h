#pragma once

class Servo {
public:
  void setPeriodHertz(int hz) {}
  void attach(int pin, int minUs, int maxUs) {}
  void write(int value) {}
  int read() { return 0; }
};

namespace ESP32PWM {
void allocateTimer(int timer) {}
}
