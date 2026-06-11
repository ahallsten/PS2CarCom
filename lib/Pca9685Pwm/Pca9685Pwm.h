#ifndef PCA9685_PWM_H
#define PCA9685_PWM_H

#include <Arduino.h>

struct Pca9685PwmSnapshot {
  bool valid = false;
  uint8_t channel = 0;
  uint8_t duty = 0;
  uint16_t ticks = 0;
};

class Pca9685Pwm {
public:
  /** Use the PCA9685 at the supplied I2C address. */
  explicit Pca9685Pwm(uint8_t address = 0x40);

  /** Initialize the PCA9685 and clear all 16 outputs low. */
  void begin(uint16_t frequencyHz);

  /** Set one PCA9685 output to an 8-bit duty cycle mapped onto 12-bit ticks. */
  void setDuty(uint8_t channel, uint8_t duty);

  /** Immediately command one PCA9685 output low. */
  void forceLow(uint8_t channel);

  /** Return the last duty commanded for a channel. */
  bool getChannelSnapshot(uint8_t channel, Pca9685PwmSnapshot &snapshot) const;

  /** Convert 0..255 duty to PCA9685 0..4095 tick units. */
  static uint16_t dutyToTicks(uint8_t duty);

private:
  uint8_t _address;
  uint8_t _lastDuty[16];
  uint16_t _lastTicks[16];
  bool _channelSeen[16];

  void write8(uint8_t reg, uint8_t value);
  uint8_t read8(uint8_t reg);
  void setPwmTicks(uint8_t channel, uint16_t on, uint16_t off);
};

#endif
