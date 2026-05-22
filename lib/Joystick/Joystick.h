#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include "AxisMap.h"

/**
 * @brief Simple analog joystick reader.
 *
 * This helper reads four analog pins and stores selected axes as 8-bit values.
 * It is currently not used by the active transmitter or receiver firmware, but
 * remains available for experiments or alternate controller hardware.
 */
class Joystick {
public:
  /**
   * @brief Create a joystick reader for four analog pins.
   *
   * @param leftXPin Analog pin for left stick X.
   * @param leftYPin Analog pin for left stick Y.
   * @param rightXPin Analog pin for right stick X.
   * @param rightYPin Analog pin for right stick Y.
   */
  Joystick(uint8_t leftXPin, uint8_t leftYPin, uint8_t rightXPin, uint8_t rightYPin);

  /** @brief Refresh cached axis values from analogRead(). */
  void update();

  /** @return Most recent left-stick Y value, scaled from 10-bit ADC to 0..255. */
  uint8_t getLeftY() const { return _leftY; }

  /** @return Most recent right-stick Y value, scaled from 10-bit ADC to 0..255. */
  uint8_t getRightY() const { return _rightY; }

  /** @return Most recent right-stick X value, scaled from 10-bit ADC to 0..255. */
  uint8_t getRightX() const { return _rightX; }

private:
  uint8_t _lxPin, _lyPin, _rxPin, _ryPin;
  uint8_t _leftY, _rightY, _rightX;
};

#endif
