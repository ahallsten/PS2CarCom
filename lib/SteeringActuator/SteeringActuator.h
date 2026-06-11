#ifndef STEERING_ACTUATOR_H
#define STEERING_ACTUATOR_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <AS5600.h>
#include "BTS7960.h"
#include "CurrentProtection.h"
#include "DriveSystem.h"

static const uint8_t STEERING_FLAG_CALIBRATION_VALID = 0x01;
static const uint8_t STEERING_FLAG_HOMED = 0x02;
static const uint8_t STEERING_FLAG_HOME_SWITCH_ACTIVE = 0x04;
static const uint8_t STEERING_FLAG_CURRENT_LIMIT_ACTIVE = 0x08;

enum SteeringState : uint8_t {
  STEERING_STATE_DISABLED = 0,
  STEERING_STATE_NEEDS_CALIBRATION = 1,
  STEERING_STATE_NEEDS_HOME = 2,
  STEERING_STATE_HOMING_RELEASE = 3,
  STEERING_STATE_HOMING_SEEK = 4,
  STEERING_STATE_CALIBRATION_MIN = 5,
  STEERING_STATE_CALIBRATION_MAX = 6,
  STEERING_STATE_RUNNING = 7,
  STEERING_STATE_FAULT = 8,
};

enum SteeringFaultFlags : uint16_t {
  STEERING_FAULT_NONE = 0x0000,
  STEERING_FAULT_SENSOR = 0x0001,
  STEERING_FAULT_EEPROM_INVALID = 0x0002,
  STEERING_FAULT_HOME_TIMEOUT = 0x0004,
  STEERING_FAULT_HOME_TRAVEL = 0x0008,
  STEERING_FAULT_CALIBRATION_TIMEOUT = 0x0010,
  STEERING_FAULT_CALIBRATION_TRAVEL = 0x0020,
  STEERING_FAULT_CURRENT_SENSE_UNAVAILABLE = 0x0040,
  STEERING_FAULT_CURRENT_LIMIT = 0x0080,
  STEERING_FAULT_CALIBRATION_GEOMETRY = 0x0100,
};

struct SteeringTelemetry {
  uint16_t rawAngle = 0;
  int32_t positionCounts = 0;
  int32_t targetCounts = 0;
  uint8_t state = STEERING_STATE_DISABLED;
  uint8_t flags = 0;
  uint16_t faultFlags = STEERING_FAULT_NONE;
  uint16_t currentRaw = 0;
};

struct SteeringActuatorConfig {
  MotorPins motorPins;
  uint8_t homeSwitchMcpPin = 2;
  bool homeSwitchActiveLow = true;
  int8_t homingDirection = -1;
  int8_t motorDirection = 1;
  uint8_t homingPwm = 80;
  uint8_t homingReleasePwm = 70;
  uint8_t calibrationPwm = 70;
  uint8_t closedLoopMinPwm = 35;
  uint8_t closedLoopMaxPwm = 150;
  int16_t proportionalDivisorCounts = 24;
  int16_t targetDeadbandCounts = 24;
  int32_t homingMaxTravelCounts = 20000;
  int32_t calibrationMaxTravelCounts = 24000;
  int32_t softwareLimitMarginCounts = 300;
  unsigned long homingDebounceMs = 40;
  unsigned long homingTimeoutMs = 10000;
  unsigned long calibrationTimeoutMs = 15000;
  unsigned long calibrationPauseMs = 350;
  uint16_t calibrationStallRawThreshold = 760;
  uint8_t calibrationStallSamples = 4;
  unsigned long calibrationStallMinMs = 120;
  CurrentProtectionConfig currentProtection;
};

class SteeringActuator {
public:
  SteeringActuator(Adafruit_MCP23X17 &mcp,
                   Pca9685Pwm &pwm,
                   const SteeringActuatorConfig &config);

  void begin();

  void update(bool outputAllowed,
              int16_t stickCommand,
              bool homeRequested,
              bool calibrationRequested,
              bool homeInterruptPending);

  void coast();
  int16_t lastCommand() const;
  void readCurrentSense(uint16_t &lisOut, uint16_t &risOut) const;
  void getTelemetry(SteeringTelemetry &telemetry) const;

private:
  struct CalibrationData {
    char magic[4];
    uint8_t version;
    uint8_t structSize;
    int32_t homeOffsetCounts;
    int32_t mechanicalMinCounts;
    int32_t mechanicalMaxCounts;
    int32_t softwareMinCounts;
    int32_t softwareMaxCounts;
    int32_t centerCounts;
    uint16_t rawAngleAtSave;
    uint8_t flags;
    uint16_t checksum;
  };

  void updateSensor();
  void updateHomeSwitch(bool clearInterrupt);
  bool readHomeSwitch();
  bool loadCalibration();
  void saveCalibration();
  bool calibrationIsSane(const CalibrationData &calibration) const;
  uint16_t checksumCalibration(const CalibrationData &calibration) const;
  void setFault(uint16_t fault);
  void startHoming(bool forCalibration);
  void serviceHoming(bool outputAllowed);
  void startCalibration();
  void serviceCalibration(bool outputAllowed);
  void runClosedLoop(bool outputAllowed, int16_t stickCommand);
  void driveSigned(int16_t pwm);
  void stopOutput();
  int32_t currentPositionCounts() const;
  int32_t mapStickToTarget(int16_t stickCommand) const;
  uint16_t maxCurrentRaw() const;
  bool currentSenseMapped() const;
  void resetStallDetector();
  bool stallDetected(uint16_t currentRaw, unsigned long nowMs);

  Adafruit_MCP23X17 *_mcp;
  BTS7960 _motor;
  AS5600 _encoder;
  SteeringActuatorConfig _config;
  CurrentProtection _protection;
  CalibrationData _calibration;

  bool _sensorOk = false;
  bool _calibrationValid = false;
  bool _homed = false;
  bool _homingForCalibration = false;
  bool _homeSwitchActive = false;
  bool _homeSwitchCandidate = false;
  bool _lastHomeSwitchActive = false;
  bool _rawValid = false;
  uint16_t _rawAngle = 0;
  uint16_t _lastRawAngle = 0;
  int32_t _encoderCounts = 0;
  int32_t _homeReferenceCounts = 0;
  int32_t _stateStartCounts = 0;
  int32_t _calibrationMinCounts = 0;
  int32_t _targetCounts = 0;
  uint16_t _currentLis = 0;
  uint16_t _currentRis = 0;
  uint16_t _filteredCalibrationCurrent = 0;
  uint8_t _stallSampleCount = 0;
  unsigned long _stateStartMs = 0;
  unsigned long _homeSwitchCandidateSinceMs = 0;
  unsigned long _calibrationPauseUntilMs = 0;
  unsigned long _stallSinceMs = 0;
  int16_t _lastCommand = 0;
  uint16_t _faultFlags = STEERING_FAULT_NONE;
  SteeringState _state = STEERING_STATE_DISABLED;
};

#endif
