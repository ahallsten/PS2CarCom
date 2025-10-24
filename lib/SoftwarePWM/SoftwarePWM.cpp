#include "SoftwarePWM.h"

SoftwarePWM::SoftwarePWM(uint8_t maxChannels) {
  _max = maxChannels;
  _ch = (PWMChannel *)malloc(sizeof(PWMChannel) * _max);
  for (uint8_t i = 0; i < _max; ++i) {
    _ch[i].pin = 255; // mark unused
  }
}

SoftwarePWM::~SoftwarePWM() {
  free(_ch);
}

int8_t SoftwarePWM::addChannel(uint8_t pin, uint8_t duty, unsigned long period_us) {
  for (uint8_t i = 0; i < _max; ++i) {
    if (_ch[i].pin == 255) {
      _ch[i].pin = pin;
      pinMode(pin, OUTPUT);
      _ch[i].duty = duty;
      _ch[i].period_us = period_us;
      _ch[i].on_time_us = dutyToMicros(duty, period_us);
      _ch[i].last_time_us = micros();
      _ch[i].state = false;
      digitalWrite(pin, LOW);
      return (int8_t)i;
    }
  }
  return -1;
}

void SoftwarePWM::removeChannel(int8_t idx) {
  if (idx < 0 || idx >= _max) return;
  if (_ch[idx].pin != 255) {
    digitalWrite(_ch[idx].pin, LOW);
    _ch[idx].pin = 255;
  }
}

void SoftwarePWM::setDuty(int8_t idx, uint8_t duty) {
  if (idx < 0 || idx >= _max) return;
  if (_ch[idx].pin == 255) return;
  _ch[idx].duty = duty;
  _ch[idx].on_time_us = dutyToMicros(duty, _ch[idx].period_us);
}

void SoftwarePWM::setPeriod(int8_t idx, unsigned long period_us) {
  if (idx < 0 || idx >= _max) return;
  if (_ch[idx].pin == 255) return;
  _ch[idx].period_us = period_us;
  _ch[idx].on_time_us = dutyToMicros(_ch[idx].duty, period_us);
}

unsigned long SoftwarePWM::dutyToMicros(uint8_t duty, unsigned long period_us) {
  // duty is 0..255
  // return on_time microseconds
  return (unsigned long)((((unsigned long)duty) * (unsigned long)period_us) / 255UL);
}

void SoftwarePWM::update() {
  unsigned long now = micros();
  for (uint8_t i = 0; i < _max; ++i) {
    if (_ch[i].pin == 255) continue;
    PWMChannel &c = _ch[i];

    // If zero duty -> ensure LOW and continue
    if (c.duty == 0) {
      if (c.state) {
        digitalWrite(c.pin, LOW);
        c.state = false;
      }
      c.last_time_us = now; // keep phase in sync
      continue;
    }
    // If full duty -> ensure HIGH and continue
    if (c.duty >= 255) {
      if (!c.state) {
        digitalWrite(c.pin, HIGH);
        c.state = true;
      }
      c.last_time_us = now;
      continue;
    }

    unsigned long elapsed = now - c.last_time_us;

    if (!c.state) {
      // low -> check if we should turn on for on_time_us
      // We treat the "off segment" as (period_on = period - on_time)
      unsigned long off_time = c.period_us - c.on_time_us;
      if (elapsed >= off_time) {
        digitalWrite(c.pin, HIGH);
        c.state = true;
        // reset timer so on-time counts from now
        c.last_time_us = now;
      }
    } else {
      // currently HIGH -> check if on-time expired
      if (elapsed >= c.on_time_us) {
        digitalWrite(c.pin, LOW);
        c.state = false;
        c.last_time_us = now;
      }
    }
  }
}
