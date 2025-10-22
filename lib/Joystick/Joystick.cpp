#include "Joystick.h"

Joystick::Joystick(uint8_t lx, uint8_t ly, uint8_t rx, uint8_t ry)
  : _lxPin(lx), _lyPin(ly), _rxPin(rx), _ryPin(ry) {}

void Joystick::update() {
  _leftY  = analogRead(_lyPin) / 4;   // 10-bit → 8-bit
  _rightY = analogRead(_ryPin) / 4;
  _rightX = analogRead(_rxPin) / 4;
}
