#ifndef JOYSTICK_H
#define JOYSTICK_H

#pragma once
#include <Arduino.h>
#include "AxisMap.h"

class Joystick {
public:
  Joystick(uint8_t leftXPin, uint8_t leftYPin, uint8_t rightXPin, uint8_t rightYPin);
  void update();
  uint8_t getLeftY() const { return _leftY; }
  uint8_t getRightY() const { return _rightY; }
  uint8_t getRightX() const { return _rightX; }

private:
  uint8_t _lxPin, _lyPin, _rxPin, _ryPin;
  uint8_t _leftY, _rightY, _rightX;
};

#endif
