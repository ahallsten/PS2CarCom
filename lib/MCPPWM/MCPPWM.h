#pragma once
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

class MCPPWM {
public:
  // levels = PWM resolution (number of "slices" per period)
  //          32 is a good start (50 Hz * 32 slices = 1600 steps/s)
  explicit MCPPWM(Adafruit_MCP23X17 &mcp, uint16_t hz = 50, uint8_t levels = 32)
  : _mcp(mcp) { setFreq(hz, levels); }

  int8_t add(uint8_t mcpPin, uint8_t duty = 0) {
    if (_n >= MAX_CH) return -1;
    _mcp.pinMode(mcpPin, OUTPUT);
    _mcp.digitalWrite(mcpPin, LOW);
    _ch[_n] = { mcpPin, dutyToSteps(duty), false };
    return _n++;
  }

  void set(int8_t ch, uint8_t duty) {
    if (ch < 0 || ch >= _n) return;
    _ch[ch].steps = dutyToSteps(duty);
  }

  void setFreq(uint16_t hz, uint8_t levels) {
    if (hz == 0) hz = 50;
    if (levels < 2) levels = 2;
    _levels = levels;
    _step_us = (uint32_t)(1000000UL / (uint32_t)hz) / _levels; // µs per sub-step
    _step = 0;
    _last = micros();
  }

  // Call this as often as you can (every loop)
  void update() {
    uint32_t now = micros();
    if ((uint32_t)(now - _last) < _step_us) return;
    _last += _step_us;
    _step = (uint8_t)((_step + 1) % _levels);

    // Edge-only writes: each channel toggles at most once per sub-step
    for (uint8_t i = 0; i < _n; ++i) {
      bool on = (_ch[i].steps > _step); // simple phase-slice comparator
      if (on != _ch[i].state) {
        _mcp.digitalWrite(_ch[i].pin, on);
        _ch[i].state = on;
      }
    }
  }

private:
  struct Channel { uint8_t pin; uint8_t steps; bool state; };
  static const uint8_t MAX_CH = 12;

  uint8_t dutyToSteps(uint8_t duty) const {
    // map 0..255 -> 0.._levels
    return (uint16_t)duty * _levels / 255;
  }

  Adafruit_MCP23X17 &_mcp;
  Channel _ch[MAX_CH];
  uint8_t _n = 0;

  uint8_t _levels = 32;
  uint32_t _step_us = 625; // default 50 Hz, 32 levels
  uint8_t _step = 0;
  uint32_t _last = 0;
};
