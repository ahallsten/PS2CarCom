#include "DriveSystem.h"

DriveSystem::DriveSystem(Adafruit_MCP23X17 &mcp,
                         Pca9685Pwm &pwm,
                         const MotorPins &fl,
                         const MotorPins &fr,
                         const MotorPins &rl,
                         const MotorPins &rr,
                         const MotorPins &steer)
  : _fl(&mcp, &pwm, fl.rpwmChannel, fl.lpwmChannel, fl.len, fl.ren, fl.lis, fl.ris),
    _fr(&mcp, &pwm, fr.rpwmChannel, fr.lpwmChannel, fr.len, fr.ren, fr.lis, fr.ris),
    _rl(&mcp, &pwm, rl.rpwmChannel, rl.lpwmChannel, rl.len, rl.ren, rl.lis, rl.ris),
    _rr(&mcp, &pwm, rr.rpwmChannel, rr.lpwmChannel, rr.len, rr.ren, rr.lis, rr.ris),
    _steer(&mcp, &pwm, steer.rpwmChannel, steer.lpwmChannel,
           steer.len, steer.ren, steer.lis, steer.ris) {
}

void DriveSystem::begin() {
  _fl.begin();
  _fr.begin();
  _rl.begin();
  _rr.begin();
  _steer.begin();
}

void DriveSystem::setEnabled(bool enabled) {
  if (_enabled == enabled) return;
  _enabled = enabled;
  if (_enabled) {
    enableAll();
  } else {
    coastAll();
    recordCmds(0, 0, 0, 0, 0);
  }
}

void DriveSystem::setParkingBrake(bool enabled) {
  if (_parkingBrake == enabled) return;
  _parkingBrake = enabled;
  if (_parkingBrake) {
    brakeAll();
    recordCmds(0, 0, 0, 0, 0);
  }
}

void DriveSystem::applyDrive(int16_t fl, int16_t fr, int16_t rl, int16_t rr, int16_t steer) {
  if (!_enabled || _parkingBrake) {
    recordCmds(0, 0, 0, 0, 0);
    return;
  }

  recordCmds(fl, fr, rl, rr, steer);
  _fl.drive(fl);
  _fr.drive(fr);
  _rl.drive(rl);
  _rr.drive(rr);
  _steer.drive(steer);
}

void DriveSystem::applyDriveAll(int16_t pwm) {
  applyDrive(pwm, pwm, pwm, pwm, pwm);
}

void DriveSystem::brake() {
  brakeAll();
  recordCmds(0, 0, 0, 0, 0);
}

void DriveSystem::coast() {
  coastAll();
  recordCmds(0, 0, 0, 0, 0);
}

void DriveSystem::stop() {
  brakeAll();
  coastAll();
  recordCmds(0, 0, 0, 0, 0);
}

void DriveSystem::enableAll() {
  _fl.enable();
  _fr.enable();
  _rl.enable();
  _rr.enable();
  _steer.enable();
}

void DriveSystem::coastAll() {
  _fl.coast();
  _fr.coast();
  _rl.coast();
  _rr.coast();
  _steer.coast();
}

void DriveSystem::brakeAll() {
  _fl.brake();
  _fr.brake();
  _rl.brake();
  _rr.brake();
  _steer.brake();
}

void DriveSystem::recordCmds(int16_t fl, int16_t fr, int16_t rl, int16_t rr, int16_t steer) {
  _lastCmd[MOTOR_INDEX_FL] = fl;
  _lastCmd[MOTOR_INDEX_FR] = fr;
  _lastCmd[MOTOR_INDEX_RL] = rl;
  _lastCmd[MOTOR_INDEX_RR] = rr;
  _lastCmd[MOTOR_INDEX_STEER] = steer;
}

void DriveSystem::getMotorCommands(int16_t out[VEHICLE_MOTOR_COUNT]) const {
  for (uint8_t i = 0; i < VEHICLE_MOTOR_COUNT; ++i) {
    out[i] = _lastCmd[i];
  }
}

void DriveSystem::getPwmSnapshots(Bts7960PwmSnapshot out[VEHICLE_MOTOR_COUNT]) const {
  _fl.getPwmSnapshot(out[MOTOR_INDEX_FL]);
  _fr.getPwmSnapshot(out[MOTOR_INDEX_FR]);
  _rl.getPwmSnapshot(out[MOTOR_INDEX_RL]);
  _rr.getPwmSnapshot(out[MOTOR_INDEX_RR]);
  _steer.getPwmSnapshot(out[MOTOR_INDEX_STEER]);
}

void DriveSystem::readCurrentSense(uint16_t out[VEHICLE_CURRENT_SENSE_COUNT]) const {
  _fl.readCurrentSense(out[CURRENT_INDEX_FL_L], out[CURRENT_INDEX_FL_R]);
  _fr.readCurrentSense(out[CURRENT_INDEX_FR_L], out[CURRENT_INDEX_FR_R]);
  _rl.readCurrentSense(out[CURRENT_INDEX_RL_L], out[CURRENT_INDEX_RL_R]);
  _rr.readCurrentSense(out[CURRENT_INDEX_RR_L], out[CURRENT_INDEX_RR_R]);
  _steer.readCurrentSense(out[CURRENT_INDEX_STEER_L], out[CURRENT_INDEX_STEER_R]);
}
