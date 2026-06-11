#ifndef PCA9685_PWM_H
#define PCA9685_PWM_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class Pca9685Pwm {
public:
  /** Wrap an Adafruit PCA9685 driver object owned by the caller. */
  explicit Pca9685Pwm(Adafruit_PWMServoDriver &driver);

  /** Initialize the PCA9685 and clear all 16 outputs low. */
  void begin(float frequencyHz);

  /** Set one PCA9685 output to an 8-bit duty cycle mapped onto 12-bit ticks. */
  void setDuty(uint8_t channel, uint8_t duty);

  /** Immediately command one PCA9685 output low. */
  void forceLow(uint8_t channel);

  /** Convert 0..255 duty to PCA9685 0..4095 tick units. */
  static uint16_t dutyToTicks(uint8_t duty);

private:
  Adafruit_PWMServoDriver *_driver;
};

#endif
