#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>

class MotorDriver {
public:
  MotorDriver(uint8_t LPWM, uint8_t RPWM, uint8_t L_EN, uint8_t R_EN);
  void begin();
  void drive(int16_t pwm); // positive = forward, negative = reverse
  void brake();
  void coast();

private:
  uint8_t _LPWM, _RPWM, _L_EN, _R_EN;
};

#endif
