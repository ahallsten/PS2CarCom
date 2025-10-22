#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t LPWM, uint8_t RPWM, uint8_t L_EN, uint8_t R_EN)
  : _LPWM(LPWM), _RPWM(RPWM), _L_EN(L_EN), _R_EN(R_EN) {}

void MotorDriver::begin() {
  pinMode(_LPWM, OUTPUT);
  pinMode(_RPWM, OUTPUT);
  pinMode(_L_EN, OUTPUT);
  pinMode(_R_EN, OUTPUT);
  digitalWrite(_L_EN, HIGH);
  digitalWrite(_R_EN, HIGH);
}

void MotorDriver::drive(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    analogWrite(_LPWM, pwm);
    analogWrite(_RPWM, 0);
  } else if (pwm < 0) {
    analogWrite(_LPWM, 0);
    analogWrite(_RPWM, -pwm);
  } else {
    brake();
  }
}

void MotorDriver::brake() {
  analogWrite(_LPWM, 0);
  analogWrite(_RPWM, 0);
}

void MotorDriver::coast() {
  digitalWrite(_L_EN, LOW);
  digitalWrite(_R_EN, LOW);
}
