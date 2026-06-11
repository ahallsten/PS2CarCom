#ifndef CURRENT_PROTECTION_H
#define CURRENT_PROTECTION_H

#include <Arduino.h>

struct CurrentProtectionConfig {
  uint16_t continuousThresholdRaw = 850;
  uint16_t peakThresholdRaw = 1000;
  uint32_t accumulatorTripRaw = 6000;
  uint16_t accumulatorDecayRaw = 120;
  unsigned long cooldownMs = 1500;
};

class CurrentProtection {
public:
  explicit CurrentProtection(const CurrentProtectionConfig &config);

  void reset();
  bool update(uint16_t currentRaw, unsigned long nowMs);
  bool isActive(unsigned long nowMs);
  bool isLatched() const;
  uint32_t accumulator() const;

private:
  void trip(unsigned long nowMs);

  CurrentProtectionConfig _config;
  uint32_t _accumulator = 0;
  bool _latched = false;
  unsigned long _tripMs = 0;
};

#endif
