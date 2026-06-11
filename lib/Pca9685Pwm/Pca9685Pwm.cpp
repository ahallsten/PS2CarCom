#include "Pca9685Pwm.h"

#include <Wire.h>

static const uint16_t kPca9685MaxTicks = 4095;
static const uint32_t kPca9685OscillatorHz = 25000000UL;
static const uint8_t kPca9685Mode1 = 0x00;
static const uint8_t kPca9685Prescale = 0xFE;
static const uint8_t kPca9685Led0OnL = 0x06;
static const uint8_t kMode1Restart = 0x80;
static const uint8_t kMode1AutoIncrement = 0x20;
static const uint8_t kMode1Sleep = 0x10;

Pca9685Pwm::Pca9685Pwm(uint8_t address)
  : _address(address),
    _lastDuty{0},
    _lastTicks{0},
    _channelSeen{false} {
}

void Pca9685Pwm::begin(uint16_t frequencyHz) {
  if (frequencyHz == 0) frequencyHz = 50;
  write8(kPca9685Mode1, kMode1Restart);
  delay(10);

  uint32_t denominator = 4096UL * static_cast<uint32_t>(frequencyHz);
  uint8_t prescale =
    static_cast<uint8_t>(((kPca9685OscillatorHz + (denominator / 2UL)) / denominator) - 1UL);
  uint8_t oldMode = read8(kPca9685Mode1);
  uint8_t sleepMode = (oldMode & ~kMode1Restart) | kMode1Sleep;
  write8(kPca9685Mode1, sleepMode);
  write8(kPca9685Prescale, prescale);
  write8(kPca9685Mode1, oldMode | kMode1AutoIncrement);
  delay(5);
  write8(kPca9685Mode1, oldMode | kMode1Restart | kMode1AutoIncrement);

  for (uint8_t channel = 0; channel < 16; ++channel) {
    forceLow(channel);
  }
}

void Pca9685Pwm::setDuty(uint8_t channel, uint8_t duty) {
  if (channel >= 16) return;
  uint16_t ticks = dutyToTicks(duty);
  setPwmTicks(channel, 0, ticks);
  _lastDuty[channel] = duty;
  _lastTicks[channel] = ticks;
  _channelSeen[channel] = true;
}

void Pca9685Pwm::forceLow(uint8_t channel) {
  if (channel >= 16) return;
  setPwmTicks(channel, 0, 0);
  _lastDuty[channel] = 0;
  _lastTicks[channel] = 0;
  _channelSeen[channel] = true;
}

bool Pca9685Pwm::getChannelSnapshot(uint8_t channel, Pca9685PwmSnapshot &snapshot) const {
  snapshot = Pca9685PwmSnapshot();
  if (channel >= 16) return false;

  snapshot.valid = _channelSeen[channel];
  snapshot.channel = channel;
  snapshot.duty = _lastDuty[channel];
  snapshot.ticks = _lastTicks[channel];
  return snapshot.valid;
}

uint16_t Pca9685Pwm::dutyToTicks(uint8_t duty) {
  return static_cast<uint16_t>((static_cast<uint32_t>(duty) * kPca9685MaxTicks + 127U) / 255U);
}

void Pca9685Pwm::write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t Pca9685Pwm::read8(uint8_t reg) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(_address, static_cast<uint8_t>(1)) != 1) return 0;
  return Wire.read();
}

void Pca9685Pwm::setPwmTicks(uint8_t channel, uint16_t on, uint16_t off) {
  uint8_t reg = kPca9685Led0OnL + (4 * channel);
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(on & 0xFF));
  Wire.write(static_cast<uint8_t>(on >> 8));
  Wire.write(static_cast<uint8_t>(off & 0xFF));
  Wire.write(static_cast<uint8_t>(off >> 8));
  Wire.endTransmission();
}
