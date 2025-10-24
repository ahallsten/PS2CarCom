#pragma once
#include <Arduino.h>

struct PWMChannel {
  uint8_t pin = 255;          // invalid if 255
  uint8_t duty = 0;           // 0..255
  unsigned long period_us = 20000UL; // default 20 ms (50 Hz) - change as needed
  unsigned long on_time_us = 0; // computed
  unsigned long last_time_us = 0;
  bool state = false;         // current pin state
};

class SoftwarePWM {
public:
  SoftwarePWM(uint8_t maxChannels = 8);
  ~SoftwarePWM();

  // add channel returns index or -1 if no room
  int8_t addChannel(uint8_t pin, uint8_t duty = 0, unsigned long period_us = 20000UL);

  // remove channel by index
  void removeChannel(int8_t idx);

  // set duty 0..255
  void setDuty(int8_t idx, uint8_t duty);

  // set period in microseconds
  void setPeriod(int8_t idx, unsigned long period_us);

  // must call regularly from loop(), or from a scheduler
  void update();

  // convenience: convert pwmVal (0..255) to percent or microseconds is internal
  static unsigned long dutyToMicros(uint8_t duty, unsigned long period_us);

private:
  PWMChannel *_ch;
  uint8_t _max;
};
