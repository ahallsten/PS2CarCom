#include "BTS7960.h"
#include <Arduino.h>

BTS7960::BTS7960(Adafruit_MCP23X17 *mcp,
                 PinDef RPWM, PinDef LPWM,
                 PinDef L_EN, PinDef R_EN,
                 PinDef L_IS, PinDef R_IS)
  : _mcp(mcp),
    _RPWM(RPWM), _LPWM(LPWM),
    _L_EN(L_EN), _R_EN(R_EN),
    _L_IS(L_IS), _R_IS(R_IS) {}

void BTS7960::pinModeX(PinDef pin, uint8_t mode) {
  if (pin.source == MCP_PIN) _mcp->pinMode(pin.pin, mode);
  else pinMode(pin.pin, mode);
}

void BTS7960::digitalWriteX(PinDef pin, uint8_t value) {
  if (pin.source == MCP_PIN) _mcp->digitalWrite(pin.pin, value);
  else digitalWrite(pin.pin, value);
}

int BTS7960::analogWriteX(PinDef pin, uint8_t) {
  if (pin.source == MCP_PIN)
    return 0; // MCP can’t do analog, just return 0 or handle error
  return analogRead(pin.pin);
}

void BTS7960::begin() {
  pinModeX(_RPWM, OUTPUT);
  pinModeX(_LPWM, OUTPUT);
  pinModeX(_L_EN, OUTPUT);
  pinModeX(_R_EN, OUTPUT);
  pinModeX(_L_IS, INPUT);
  pinModeX(_R_IS, INPUT);
  coast();
}

void BTS7960::drive(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWriteX(_LPWM, pwm);
    analogWriteX(_RPWM, 0);
  } else if (pwm < 0) {
    analogWriteX(_LPWM, 0);
    analogWriteX(_RPWM, -pwm);
  } else {
    brake();
  }
}

void BTS7960::cwBrake()
{
    digitalWriteX(_R_EN, 1);
    digitalWriteX(_L_EN, 0);
}

void BTS7960::ccwBrake()
{
    digitalWriteX(_R_EN, 0);
    digitalWriteX(_L_EN, 1);
}

void BTS7960::enable()
{
    digitalWriteX(_R_EN, 1);
    digitalWriteX(_L_EN, 1);
}

void BTS7960::brake() {
  analogWriteX(_LPWM, 0);
  analogWriteX(_RPWM, 0);
}

void BTS7960::coast() {
  digitalWriteX(_L_EN, LOW);
  digitalWriteX(_R_EN, LOW);
}