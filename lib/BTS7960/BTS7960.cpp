#include <Arduino.h>
#include "BTS7960.h"

static const unsigned int kDirectionDeadTimeUs = 200;

BTS7960::BTS7960(Adafruit_MCP23X17 *mcp,
                 Adafruit_PWMServoDriver *pwm,
                 uint8_t rPwmChannel,
                 uint8_t lPwmChannel,
                 PinDef L_EN,
                 PinDef R_EN,
                 PinDef L_IS,
                 PinDef R_IS)
  : _mcp(mcp),
    _pwm(pwm),
    _rPwmChannel(rPwmChannel),
    _lPwmChannel(lPwmChannel),
    _L_EN(L_EN), _R_EN(R_EN),
    _L_IS(L_IS), _R_IS(R_IS) {
}

void BTS7960::pinModeX(PinDef pin, uint8_t mode) {
  if (pin.source == PinSource::MCP_PIN) _mcp->pinMode(pin.pin, mode);
  else pinMode(pin.pin, mode);
}

void BTS7960::digitalWriteX(PinDef pin, uint8_t value) {
  if (pin.source == PinSource::MCP_PIN) _mcp->digitalWrite(pin.pin, value);
  else digitalWrite(pin.pin, value);
}

uint16_t BTS7960::analogReadX(PinDef pin) const {
  if (pin.source != PinSource::MCU_PIN) return 0;
  int raw = analogRead(pin.pin);
  if (raw < 0) raw = 0;
  if (raw > 1023) raw = 1023;
  return static_cast<uint16_t>(raw);
}

void BTS7960::begin() {
  pinModeX(_L_EN, OUTPUT);
  pinModeX(_R_EN, OUTPUT);
  pinModeX(_L_IS, INPUT);
  pinModeX(_R_IS, INPUT);
  coast();
}

void BTS7960::drive(int16_t pwm) {
  if (!_pwm) return;
  pwm = constrain(pwm, -4095, 4095);
  if (pwm == 0) {
    brake();
    return;
  }

//   static void writeBtsPwm(uint8_t rpwm, uint8_t lpwm, int16_t command) {
//   command = constrain(command, -4095, 4095);
//   const uint16_t duty = static_cast<uint16_t>(abs(command));

//   if (command > 0) {
//     pca9685.setPin(lpwm, 0);
//     pca9685.setPin(rpwm, duty);
//   } else if (command < 0) {
//     pca9685.setPin(rpwm, 0);
//     pca9685.setPin(lpwm, duty);
//   } else {
//     pca9685.setPin(rpwm, 0);
//     pca9685.setPin(lpwm, 0);
//   }
// }

  int8_t dir = (pwm > 0) ? 1 : -1;
  uint16_t duty = static_cast<uint16_t>(abs(pwm));

  if (dir != _lastDir) {
    _pwm->setPin(_rPwmChannel, 0);
    _pwm->setPin(_lPwmChannel, 0);
    if (_lastDir != 0) {
      delayMicroseconds(kDirectionDeadTimeUs);
    }
  }

  if (dir > 0) {
    _pwm->setPin(_rPwmChannel, duty);
  } else {
    _pwm->setPin(_lPwmChannel, duty);
  }

  _lastDir = dir;
}

void BTS7960::brake() {
  if (!_pwm) return;
  _pwm->setPin(_rPwmChannel, 0);
  _pwm->setPin(_lPwmChannel, 0);
  _lastDir = 0;
}

void BTS7960::coast() {
  brake();
  digitalWriteX(_L_EN, LOW);
  digitalWriteX(_R_EN, LOW);
}

void BTS7960::cwBrake() {
  brake();
  digitalWriteX(_R_EN, HIGH);
  digitalWriteX(_L_EN, LOW);
}

void BTS7960::ccwBrake() {
  brake();
  digitalWriteX(_R_EN, LOW);
  digitalWriteX(_L_EN, HIGH);
}

void BTS7960::enable() {
  digitalWriteX(_R_EN, HIGH);
  digitalWriteX(_L_EN, HIGH);
}

void BTS7960::disable() {
  coast();
}

void BTS7960::stop() {
  brake();
  coast();
}

void BTS7960::readCurrentSense(uint16_t &lisOut, uint16_t &risOut) const {
  lisOut = analogReadX(_L_IS);
  risOut = analogReadX(_R_IS);
}
