#include <Arduino.h>
#include "BTS7960.h"

static const unsigned int kDirectionDeadTimeUs = 200;

BTS7960::BTS7960(Adafruit_MCP23X17 *mcp,
                 SoftwarePWMX *pwmx,
                 PinDef RPWM, PinDef LPWM,
                 PinDef L_EN, PinDef R_EN,
                 PinDef L_IS, PinDef R_IS)
  : _mcp(mcp),
    _pwmx(pwmx),
    _RPWM(RPWM), _LPWM(LPWM),
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

void BTS7960::begin() {
  pinModeX(_RPWM, OUTPUT);
  pinModeX(_LPWM, OUTPUT);
  pinModeX(_L_EN, OUTPUT);
  pinModeX(_R_EN, OUTPUT);
  pinModeX(_L_IS, INPUT);
  pinModeX(_R_IS, INPUT);

  _rPwmCh = _pwmx ? _pwmx->addChannel(_RPWM, 0) : -1;
  _lPwmCh = _pwmx ? _pwmx->addChannel(_LPWM, 0) : -1;
  coast();
}

void BTS7960::drive(int16_t pwm) {
  if (_rPwmCh < 0 || _lPwmCh < 0) return;
  pwm = constrain(pwm, -255, 255);
  if (pwm == 0) {
    brake();
    return;
  }

  int8_t dir = (pwm > 0) ? 1 : -1;
  uint8_t duty = static_cast<uint8_t>(abs(pwm));

  if (dir != _lastDir) {
    _pwmx->forceLow(_rPwmCh);
    _pwmx->forceLow(_lPwmCh);
    if (_lastDir != 0) {
      delayMicroseconds(kDirectionDeadTimeUs);
    }
  }

  if (dir > 0) {
    _pwmx->setDuty(_rPwmCh, duty);
  } else {
    _pwmx->setDuty(_lPwmCh, duty);
  }

  _lastDir = dir;
}

void BTS7960::brake() {
  if (_rPwmCh < 0 || _lPwmCh < 0) return;
  _pwmx->forceLow(_rPwmCh);
  _pwmx->forceLow(_lPwmCh);
  _lastDir = 0;
}

void BTS7960::coast() {
  brake();
  digitalWriteX(_L_EN, LOW);
  digitalWriteX(_R_EN, LOW);
}

void BTS7960::cwBrake() {
  brake();
  digitalWriteX(_R_EN, 1);
  digitalWriteX(_L_EN, 0);
}

void BTS7960::ccwBrake() {
  brake();
  digitalWriteX(_R_EN, 0);
  digitalWriteX(_L_EN, 1);
}

void BTS7960::enable() {
  digitalWriteX(_R_EN, 1);
  digitalWriteX(_L_EN, 1);
}

void BTS7960::disable() {
  coast();
}

void BTS7960::stop() {
  brake();
  coast();
}

void BTS7960::getPwmSnapshot(Bts7960PwmSnapshot &snapshot) const {
  snapshot = Bts7960PwmSnapshot();
  snapshot.direction = _lastDir;
  if (!_pwmx) return;

  _pwmx->getChannelSnapshot(_rPwmCh, snapshot.rpwm);
  _pwmx->getChannelSnapshot(_lPwmCh, snapshot.lpwm);
}
