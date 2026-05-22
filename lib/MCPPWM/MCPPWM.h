#pragma once
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>


/**
 * @brief Lightweight phase-sliced PWM generator for MCP23017 output pins.
 *
 * This experimental helper toggles MCP23017 pins in discrete sub-steps across a
 * PWM period. The active receiver firmware currently uses SoftwarePWMX instead,
 * but this class remains in the repo as an alternate MCP-only PWM approach.
 */
class MCPPWM {
public:
  /**
   * @brief Create an MCP23017 PWM scheduler.
   *
   * @param mcp MCP23017 instance that owns the output pins.
   * @param hz PWM frequency in periods per second. A value of 0 is coerced to 50.
   * @param levels Number of duty slices per period. Values below 2 are coerced to 2.
   */
  explicit MCPPWM(Adafruit_MCP23X17 &mcp, uint16_t hz = 50, uint8_t levels = 32)
  : _mcp(mcp) { setFreq(hz, levels); }

  /**
   * @brief Add an MCP23017 pin as a PWM channel.
   *
   * @param mcpPin MCP23017 pin index.
   * @param duty Initial duty value, 0..255.
   * @return Channel index, or -1 if no channel slot is available.
   */
  int8_t add(uint8_t mcpPin, uint8_t duty = 0) {
    if (_n >= MAX_CH) return -1;
    _mcp.pinMode(mcpPin, OUTPUT);
    _mcp.digitalWrite(mcpPin, LOW);
    _ch[_n] = { mcpPin, dutyToSteps(duty), false };
    return _n++;
  }

  /**
   * @brief Set a channel duty cycle.
   *
   * Invalid channel indexes are ignored.
   *
   * @param ch Channel index returned by add().
   * @param duty Duty value, 0..255.
   */
  void set(int8_t ch, uint8_t duty) {
    if (ch < 0 || ch >= _n) return;
    _ch[ch].steps = dutyToSteps(duty);
  }

  /**
   * @brief Configure PWM frequency and duty resolution.
   *
   * @param hz PWM frequency in periods per second.
   * @param levels Number of slices in each PWM period.
   */
  void setFreq(uint16_t hz, uint8_t levels) {
    if (hz == 0) hz = 50;
    if (levels < 2) levels = 2;
    _levels = levels;
    _step_us = (uint32_t)(1000000UL / (uint32_t)hz) / _levels; // µs per sub-step
    _step = 0;
    _last = micros();
  }

  /**
   * @brief Advance the PWM scheduler and toggle output pins when needed.
   *
   * Call this from loop() as often as possible. It returns immediately until the
   * next sub-step is due.
   */
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

  /**
   * @brief Convert an 8-bit duty value into the current slice count.
   *
   * @param duty Duty value, 0..255.
   * @return Number of active slices in the configured period.
   */
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
