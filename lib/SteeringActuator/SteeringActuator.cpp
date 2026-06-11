#include "SteeringActuator.h"

#include <EEPROM.h>

static const uint16_t kAs5600CountsPerTurn = 4096;
static const int16_t kAs5600HalfScale = 2048;
static const uint8_t kCalibrationVersion = 1;
static const int kCalibrationEepromAddress = 0;
static const char kCalibrationMagic[4] = {'S', 'T', 'R', '1'};

static int32_t abs32(int32_t value) {
  return value < 0 ? -value : value;
}

static int16_t clampSignedPwm(int16_t pwm) {
  if (pwm > 255) return 255;
  if (pwm < -255) return -255;
  return pwm;
}

static uint16_t maxU16(uint16_t a, uint16_t b) {
  return a > b ? a : b;
}

SteeringActuator::SteeringActuator(Adafruit_MCP23X17 &mcp,
                                   Pca9685Pwm &pwm,
                                   const SteeringActuatorConfig &config)
  : _mcp(&mcp),
    _motor(&mcp, &pwm,
           config.motorPins.rpwmChannel,
           config.motorPins.lpwmChannel,
           config.motorPins.len,
           config.motorPins.ren,
           config.motorPins.lis,
           config.motorPins.ris),
    _encoder(),
    _config(config),
    _protection(config.currentProtection),
    _calibration{} {
}

void SteeringActuator::begin() {
  _motor.begin();
  _motor.disable();
  _mcp->pinMode(_config.homeSwitchMcpPin, INPUT_PULLUP);
  _homeSwitchActive = readHomeSwitch();

  _sensorOk = _encoder.begin() && _encoder.isConnected();
  if (_sensorOk) {
    updateSensor();
  } else {
    _faultFlags |= STEERING_FAULT_SENSOR;
  }

  _calibrationValid = loadCalibration();
  if (!_calibrationValid) {
    _faultFlags |= STEERING_FAULT_EEPROM_INVALID;
  }

  if (!_sensorOk) {
    _state = STEERING_STATE_FAULT;
  } else if (_calibrationValid) {
    _state = STEERING_STATE_NEEDS_HOME;
  } else {
    _state = STEERING_STATE_NEEDS_CALIBRATION;
  }
}

void SteeringActuator::update(bool outputAllowed,
                              int16_t stickCommand,
                              bool homeRequested,
                              bool calibrationRequested,
                              bool homeInterruptPending) {
  unsigned long now = millis();
  updateSensor();
  updateHomeSwitch(homeInterruptPending);
  _motor.readCurrentSense(_currentLis, _currentRis);

  bool protectionActive = _protection.update(maxCurrentRaw(), now);
  if (protectionActive) {
    _faultFlags |= STEERING_FAULT_CURRENT_LIMIT;
    coast();
  } else {
    _faultFlags &= ~STEERING_FAULT_CURRENT_LIMIT;
  }

  if (!_sensorOk) {
    _faultFlags |= STEERING_FAULT_SENSOR;
    _state = STEERING_STATE_FAULT;
    coast();
    return;
  }

  if (protectionActive) return;

  if (calibrationRequested && outputAllowed &&
      _state != STEERING_STATE_CALIBRATION_MIN &&
      _state != STEERING_STATE_CALIBRATION_MAX) {
    startCalibration();
  } else if (homeRequested && outputAllowed && _calibrationValid &&
             _state != STEERING_STATE_HOMING_RELEASE &&
             _state != STEERING_STATE_HOMING_SEEK &&
             _state != STEERING_STATE_CALIBRATION_MIN &&
             _state != STEERING_STATE_CALIBRATION_MAX) {
    startHoming(false);
  }

  switch (_state) {
    case STEERING_STATE_HOMING_RELEASE:
    case STEERING_STATE_HOMING_SEEK:
      serviceHoming(outputAllowed);
      break;
    case STEERING_STATE_CALIBRATION_MIN:
    case STEERING_STATE_CALIBRATION_MAX:
      serviceCalibration(outputAllowed);
      break;
    case STEERING_STATE_RUNNING:
    case STEERING_STATE_DISABLED:
      runClosedLoop(outputAllowed, stickCommand);
      break;
    case STEERING_STATE_NEEDS_HOME:
    case STEERING_STATE_NEEDS_CALIBRATION:
    case STEERING_STATE_FAULT:
    default:
      coast();
      break;
  }
}

void SteeringActuator::coast() {
  _motor.coast();
  _lastCommand = 0;
}

int16_t SteeringActuator::lastCommand() const {
  return _lastCommand;
}

void SteeringActuator::readCurrentSense(uint16_t &lisOut, uint16_t &risOut) const {
  lisOut = _currentLis;
  risOut = _currentRis;
}

void SteeringActuator::getTelemetry(SteeringTelemetry &telemetry) const {
  telemetry.rawAngle = _rawAngle;
  telemetry.positionCounts = currentPositionCounts();
  telemetry.targetCounts = _targetCounts;
  telemetry.state = static_cast<uint8_t>(_state);
  telemetry.flags = 0;
  if (_calibrationValid) telemetry.flags |= STEERING_FLAG_CALIBRATION_VALID;
  if (_homed) telemetry.flags |= STEERING_FLAG_HOMED;
  if (_homeSwitchActive) telemetry.flags |= STEERING_FLAG_HOME_SWITCH_ACTIVE;
  if ((_faultFlags & STEERING_FAULT_CURRENT_LIMIT) != 0) {
    telemetry.flags |= STEERING_FLAG_CURRENT_LIMIT_ACTIVE;
  }
  telemetry.faultFlags = _faultFlags;
  telemetry.currentRaw = maxCurrentRaw();
}

void SteeringActuator::updateSensor() {
  if (!_sensorOk) {
    _sensorOk = _encoder.isConnected();
    if (!_sensorOk) return;
  }

  uint16_t raw = _encoder.readAngle();
  if (raw >= kAs5600CountsPerTurn) {
    _sensorOk = false;
    return;
  }

  _rawAngle = raw;
  if (!_rawValid) {
    _lastRawAngle = raw;
    _encoderCounts = raw;
    _homeReferenceCounts = _encoderCounts;
    _rawValid = true;
    return;
  }

  int16_t delta = static_cast<int16_t>(raw) - static_cast<int16_t>(_lastRawAngle);
  if (delta > kAs5600HalfScale) {
    delta -= kAs5600CountsPerTurn;
  } else if (delta < -kAs5600HalfScale) {
    delta += kAs5600CountsPerTurn;
  }

  _encoderCounts += delta;
  _lastRawAngle = raw;
}

void SteeringActuator::updateHomeSwitch(bool clearInterrupt) {
  _homeSwitchActive = readHomeSwitch();
  if (clearInterrupt) {
    _mcp->clearInterrupts();
  }
}

bool SteeringActuator::readHomeSwitch() {
  bool rawHigh = _mcp->digitalRead(_config.homeSwitchMcpPin) != LOW;
  return _config.homeSwitchActiveLow ? !rawHigh : rawHigh;
}

bool SteeringActuator::loadCalibration() {
  CalibrationData candidate = {};
  EEPROM.get(kCalibrationEepromAddress, candidate);
  if (candidate.magic[0] != kCalibrationMagic[0] ||
      candidate.magic[1] != kCalibrationMagic[1] ||
      candidate.magic[2] != kCalibrationMagic[2] ||
      candidate.magic[3] != kCalibrationMagic[3]) {
    return false;
  }
  if (candidate.version != kCalibrationVersion ||
      candidate.structSize != sizeof(CalibrationData)) {
    return false;
  }

  uint16_t expected = candidate.checksum;
  candidate.checksum = 0;
  if (checksumCalibration(candidate) != expected) {
    return false;
  }
  candidate.checksum = expected;
  if (!calibrationIsSane(candidate)) return false;

  _calibration = candidate;
  return true;
}

void SteeringActuator::saveCalibration() {
  _calibration.checksum = 0;
  _calibration.checksum = checksumCalibration(_calibration);
  EEPROM.put(kCalibrationEepromAddress, _calibration);
}

bool SteeringActuator::calibrationIsSane(const CalibrationData &calibration) const {
  if (calibration.mechanicalMinCounts >= calibration.mechanicalMaxCounts) return false;
  if (calibration.softwareMinCounts >= calibration.softwareMaxCounts) return false;
  if (calibration.softwareMinCounts < calibration.mechanicalMinCounts) return false;
  if (calibration.softwareMaxCounts > calibration.mechanicalMaxCounts) return false;
  if (calibration.centerCounts <= calibration.softwareMinCounts) return false;
  if (calibration.centerCounts >= calibration.softwareMaxCounts) return false;
  return true;
}

uint16_t SteeringActuator::checksumCalibration(const CalibrationData &calibration) const {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&calibration);
  const size_t len = sizeof(CalibrationData) - sizeof(calibration.checksum);
  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (size_t i = 0; i < len; ++i) {
    sum1 = (sum1 + bytes[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }

  return static_cast<uint16_t>((sum2 << 8) | sum1);
}

void SteeringActuator::setFault(uint16_t fault) {
  _faultFlags |= fault;
  _state = STEERING_STATE_FAULT;
  coast();
}

void SteeringActuator::startHoming(bool forCalibration) {
  unsigned long now = millis();
  _homed = false;
  _homingForCalibration = forCalibration;
  _state = _homeSwitchActive ? STEERING_STATE_HOMING_RELEASE : STEERING_STATE_HOMING_SEEK;
  _stateStartMs = now;
  _stateStartCounts = _encoderCounts;
  _homeSwitchCandidate = false;
  _homeSwitchCandidateSinceMs = now;
  _faultFlags &= ~(STEERING_FAULT_HOME_TIMEOUT | STEERING_FAULT_HOME_TRAVEL);
}

void SteeringActuator::serviceHoming(bool outputAllowed) {
  unsigned long now = millis();
  if (!outputAllowed) {
    coast();
    _state = _homingForCalibration ? STEERING_STATE_NEEDS_CALIBRATION : STEERING_STATE_NEEDS_HOME;
    return;
  }

  if (now - _stateStartMs > _config.homingTimeoutMs) {
    setFault(STEERING_FAULT_HOME_TIMEOUT);
    return;
  }
  if (abs32(_encoderCounts - _stateStartCounts) > _config.homingMaxTravelCounts) {
    setFault(STEERING_FAULT_HOME_TRAVEL);
    return;
  }

  bool wantActive = (_state == STEERING_STATE_HOMING_SEEK);
  bool atDesiredSwitchState = (_homeSwitchActive == wantActive);
  if (atDesiredSwitchState) {
    if (!_homeSwitchCandidate || _lastHomeSwitchActive != _homeSwitchActive) {
      _homeSwitchCandidate = true;
      _lastHomeSwitchActive = _homeSwitchActive;
      _homeSwitchCandidateSinceMs = now;
    } else if (now - _homeSwitchCandidateSinceMs >= _config.homingDebounceMs) {
      if (_state == STEERING_STATE_HOMING_RELEASE) {
        _state = STEERING_STATE_HOMING_SEEK;
        _stateStartMs = now;
        _stateStartCounts = _encoderCounts;
        _homeSwitchCandidate = false;
      } else {
        _homeReferenceCounts = _encoderCounts;
        _homed = true;
        _targetCounts = currentPositionCounts();
        coast();
        if (_homingForCalibration) {
          _state = STEERING_STATE_CALIBRATION_MIN;
          _stateStartMs = now;
          _stateStartCounts = currentPositionCounts();
          _calibrationPauseUntilMs = now + _config.calibrationPauseMs;
          resetStallDetector();
        } else {
          _state = _calibrationValid ? STEERING_STATE_RUNNING : STEERING_STATE_NEEDS_CALIBRATION;
        }
      }
    }
  } else {
    _homeSwitchCandidate = false;
  }

  if (_state == STEERING_STATE_HOMING_RELEASE) {
    driveSigned(-_config.homingDirection * static_cast<int16_t>(_config.homingReleasePwm));
  } else if (_state == STEERING_STATE_HOMING_SEEK) {
    driveSigned(_config.homingDirection * static_cast<int16_t>(_config.homingPwm));
  }
}

void SteeringActuator::startCalibration() {
  if (!currentSenseMapped()) {
    setFault(STEERING_FAULT_CURRENT_SENSE_UNAVAILABLE);
    return;
  }

  _calibrationValid = false;
  _faultFlags &= ~(STEERING_FAULT_EEPROM_INVALID |
                   STEERING_FAULT_HOME_TIMEOUT |
                   STEERING_FAULT_HOME_TRAVEL |
                   STEERING_FAULT_CALIBRATION_TIMEOUT |
                   STEERING_FAULT_CALIBRATION_TRAVEL |
                   STEERING_FAULT_CALIBRATION_GEOMETRY);
  startHoming(true);
}

void SteeringActuator::serviceCalibration(bool outputAllowed) {
  unsigned long now = millis();
  if (!outputAllowed) {
    coast();
    _state = STEERING_STATE_NEEDS_CALIBRATION;
    return;
  }

  if (now < _calibrationPauseUntilMs) {
    coast();
    return;
  }

  int32_t pos = currentPositionCounts();
  if (now - _stateStartMs > _config.calibrationTimeoutMs) {
    setFault(STEERING_FAULT_CALIBRATION_TIMEOUT);
    return;
  }
  if (abs32(pos - _stateStartCounts) > _config.calibrationMaxTravelCounts) {
    setFault(STEERING_FAULT_CALIBRATION_TRAVEL);
    return;
  }

  int8_t direction = (_state == STEERING_STATE_CALIBRATION_MIN) ? -1 : 1;
  driveSigned(direction * static_cast<int16_t>(_config.calibrationPwm));

  if (!stallDetected(maxCurrentRaw(), now)) return;

  coast();
  if (_state == STEERING_STATE_CALIBRATION_MIN) {
    _calibrationMinCounts = pos;
    _state = STEERING_STATE_CALIBRATION_MAX;
    _stateStartMs = now;
    _stateStartCounts = pos;
    _calibrationPauseUntilMs = now + _config.calibrationPauseMs;
    resetStallDetector();
    return;
  }

  int32_t mechanicalMin = _calibrationMinCounts;
  int32_t mechanicalMax = pos;
  if (mechanicalMin > mechanicalMax) {
    int32_t tmp = mechanicalMin;
    mechanicalMin = mechanicalMax;
    mechanicalMax = tmp;
  }

  int32_t travel = mechanicalMax - mechanicalMin;
  int32_t margin = _config.softwareLimitMarginCounts;
  if (margin * 2 >= travel) {
    margin = travel / 4;
  }
  if (travel < 200 || margin < 1) {
    setFault(STEERING_FAULT_CALIBRATION_GEOMETRY);
    return;
  }

  CalibrationData next = {};
  next.magic[0] = kCalibrationMagic[0];
  next.magic[1] = kCalibrationMagic[1];
  next.magic[2] = kCalibrationMagic[2];
  next.magic[3] = kCalibrationMagic[3];
  next.version = kCalibrationVersion;
  next.structSize = sizeof(CalibrationData);
  next.homeOffsetCounts = 0;
  next.mechanicalMinCounts = mechanicalMin;
  next.mechanicalMaxCounts = mechanicalMax;
  next.softwareMinCounts = mechanicalMin + margin;
  next.softwareMaxCounts = mechanicalMax - margin;
  next.centerCounts = mechanicalMin + (travel / 2);
  next.rawAngleAtSave = _rawAngle;
  next.flags = 0;

  if (!calibrationIsSane(next)) {
    setFault(STEERING_FAULT_CALIBRATION_GEOMETRY);
    return;
  }

  _calibration = next;
  saveCalibration();
  _calibrationValid = true;
  _faultFlags &= ~(STEERING_FAULT_EEPROM_INVALID |
                   STEERING_FAULT_CALIBRATION_TIMEOUT |
                   STEERING_FAULT_CALIBRATION_TRAVEL |
                   STEERING_FAULT_CALIBRATION_GEOMETRY);
  _targetCounts = _calibration.centerCounts;
  _state = STEERING_STATE_RUNNING;
}

void SteeringActuator::runClosedLoop(bool outputAllowed, int16_t stickCommand) {
  if (!outputAllowed) {
    _state = STEERING_STATE_DISABLED;
    coast();
    return;
  }
  if (!_calibrationValid) {
    _state = STEERING_STATE_NEEDS_CALIBRATION;
    coast();
    return;
  }
  if (!_homed) {
    _state = STEERING_STATE_NEEDS_HOME;
    coast();
    return;
  }

  _state = STEERING_STATE_RUNNING;
  _targetCounts = mapStickToTarget(stickCommand);
  int32_t pos = currentPositionCounts();
  int32_t error = _targetCounts - pos;
  if (abs32(error) <= _config.targetDeadbandCounts) {
    _motor.brake();
    _lastCommand = 0;
    return;
  }

  int16_t magnitude =
    static_cast<int16_t>(abs32(error) / _config.proportionalDivisorCounts) +
    _config.closedLoopMinPwm;
  magnitude = constrain(magnitude, _config.closedLoopMinPwm, _config.closedLoopMaxPwm);

  int16_t command = (error > 0) ? magnitude : -magnitude;
  if ((pos <= _calibration.softwareMinCounts && command < 0) ||
      (pos >= _calibration.softwareMaxCounts && command > 0)) {
    coast();
    return;
  }

  driveSigned(command);
}

void SteeringActuator::driveSigned(int16_t pwm) {
  int16_t command = clampSignedPwm(pwm * _config.motorDirection);
  _motor.enable();
  _motor.drive(command);
  _lastCommand = command;
}

int32_t SteeringActuator::currentPositionCounts() const {
  return _encoderCounts - _homeReferenceCounts - _calibration.homeOffsetCounts;
}

int32_t SteeringActuator::mapStickToTarget(int16_t stickCommand) const {
  int16_t stick = clampSignedPwm(stickCommand);
  if (!_calibrationValid) return 0;

  if (stick >= 0) {
    int32_t span = _calibration.softwareMaxCounts - _calibration.centerCounts;
    return _calibration.centerCounts + ((span * stick) / 255);
  }

  int32_t span = _calibration.centerCounts - _calibration.softwareMinCounts;
  return _calibration.centerCounts + ((span * stick) / 255);
}

uint16_t SteeringActuator::maxCurrentRaw() const {
  return maxU16(_currentLis, _currentRis);
}

bool SteeringActuator::currentSenseMapped() const {
  return _config.motorPins.lis.source == PinSource::MCU_PIN &&
         _config.motorPins.ris.source == PinSource::MCU_PIN;
}

void SteeringActuator::resetStallDetector() {
  _filteredCalibrationCurrent = 0;
  _stallSampleCount = 0;
  _stallSinceMs = 0;
}

bool SteeringActuator::stallDetected(uint16_t currentRaw, unsigned long nowMs) {
  _filteredCalibrationCurrent =
    static_cast<uint16_t>(((static_cast<uint32_t>(_filteredCalibrationCurrent) * 3U) +
                           currentRaw) / 4U);

  if (_filteredCalibrationCurrent <= _config.calibrationStallRawThreshold) {
    _stallSampleCount = 0;
    _stallSinceMs = 0;
    return false;
  }

  if (_stallSampleCount < 255) ++_stallSampleCount;
  if (_stallSinceMs == 0) _stallSinceMs = nowMs;

  return _stallSampleCount >= _config.calibrationStallSamples &&
         (nowMs - _stallSinceMs >= _config.calibrationStallMinMs);
}
