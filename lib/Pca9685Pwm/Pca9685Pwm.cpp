#include "Pca9685Pwm.h"

static const uint16_t kPca9685MaxTicks = 4095;

Pca9685Pwm::Pca9685Pwm(Adafruit_PWMServoDriver &driver)
  : _driver(&driver) {
}

void Pca9685Pwm::begin(float frequencyHz) {
  _driver->begin();
  _driver->setPWMFreq(frequencyHz);
  for (uint8_t channel = 0; channel < 16; ++channel) {
    forceLow(channel);
  }
}

void Pca9685Pwm::setDuty(uint8_t channel, uint8_t duty) {
  _driver->setPWM(channel, 0, dutyToTicks(duty));
}

void Pca9685Pwm::forceLow(uint8_t channel) {
  _driver->setPWM(channel, 0, 0);
}

uint16_t Pca9685Pwm::dutyToTicks(uint8_t duty) {
  return static_cast<uint16_t>((static_cast<uint32_t>(duty) * kPca9685MaxTicks + 127U) / 255U);
}
