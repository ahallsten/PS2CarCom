#include "CurrentProtection.h"

CurrentProtection::CurrentProtection(const CurrentProtectionConfig &config)
  : _config(config) {
}

void CurrentProtection::reset() {
  _accumulator = 0;
  _latched = false;
  _tripMs = 0;
}

bool CurrentProtection::update(uint16_t currentRaw, unsigned long nowMs) {
  if (isActive(nowMs)) return true;

  if (currentRaw >= _config.peakThresholdRaw) {
    trip(nowMs);
    return true;
  }

  if (currentRaw > _config.continuousThresholdRaw) {
    _accumulator += static_cast<uint32_t>(currentRaw - _config.continuousThresholdRaw);
    if (_accumulator >= _config.accumulatorTripRaw) {
      trip(nowMs);
      return true;
    }
  } else if (_accumulator > _config.accumulatorDecayRaw) {
    _accumulator -= _config.accumulatorDecayRaw;
  } else {
    _accumulator = 0;
  }

  return false;
}

bool CurrentProtection::isActive(unsigned long nowMs) {
  if (!_latched) return false;
  if (nowMs - _tripMs < _config.cooldownMs) return true;

  _latched = false;
  _accumulator = 0;
  return false;
}

bool CurrentProtection::isLatched() const {
  return _latched;
}

uint32_t CurrentProtection::accumulator() const {
  return _accumulator;
}

void CurrentProtection::trip(unsigned long nowMs) {
  _latched = true;
  _tripMs = nowMs;
}
