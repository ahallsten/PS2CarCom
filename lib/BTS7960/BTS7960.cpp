/*
  BTS7960.cpp - Library to control the BTS7960 motor driver
*/
#include "BTS7960.h"

BTS7960::BTS7960(uint8_t RPWM, uint8_t LPWM, uint8_t L_EN, uint8_t R_EN, uint8_t L_IS, uint8_t R_IS)
  : _LPWM(LPWM), _RPWM(RPWM), _L_EN(L_EN), _R_EN(R_EN), _L_IS(L_IS), _R_IS(R_IS) {}

void BTS7960::begin() {
  pinMode(_LPWM, OUTPUT);
  pinMode(_RPWM, OUTPUT);
  pinMode(_L_EN, OUTPUT);
  pinMode(_R_EN, OUTPUT);
  digitalWrite(_L_EN, HIGH);
  digitalWrite(_R_EN, HIGH);
}

void BTS7960::drive(int16_t pwm) {
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

void BTS7960::cwBrake()
{
    digitalWrite(_R_EN, 1);
    digitalWrite(_L_EN, 0);
}

void BTS7960::ccwBrake()
{
    digitalWrite(_R_EN, 0);
    digitalWrite(_L_EN, 1);
}

void BTS7960::enable()
{
    digitalWrite(_R_EN, 1);
    digitalWrite(_L_EN, 1);
}

void BTS7960::brake() {
  analogWrite(_LPWM, 0);
  analogWrite(_RPWM, 0);
}

void BTS7960::coast() {
  digitalWrite(_L_EN, LOW);
  digitalWrite(_R_EN, LOW);
}
