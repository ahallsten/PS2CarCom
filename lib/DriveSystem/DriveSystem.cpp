#include "DriveSystem.h"

DriveSystem::DriveSystem(Adafruit_MCP23X17 &mcp,
                         SoftwarePWMX &pwmx,
                         const MotorPins &fl,
                         const MotorPins &fr,
                         const MotorPins &rl,
                         const MotorPins &rr)
  : _fl(&mcp, &pwmx, fl.rpwm, fl.lpwm, fl.len, fl.ren, fl.lis, fl.ris),
    _fr(&mcp, &pwmx, fr.rpwm, fr.lpwm, fr.len, fr.ren, fr.lis, fr.ris),
    _rl(&mcp, &pwmx, rl.rpwm, rl.lpwm, rl.len, rl.ren, rl.lis, rl.ris),
    _rr(&mcp, &pwmx, rr.rpwm, rr.lpwm, rr.len, rr.ren, rr.lis, rr.ris) {
}

void DriveSystem::begin() {
  _fl.begin();
  _fr.begin();
  _rl.begin();
  _rr.begin();
}

void DriveSystem::setEnabled(bool enabled) {
  _enabled = enabled;
  if (_enabled) enableAll();
  else coastAll();
}

void DriveSystem::setParkingBrake(bool enabled) {
  _parkingBrake = enabled;
  if (_parkingBrake) brakeAll();
}

void DriveSystem::applyDrive(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
  if (_parkingBrake) {
    brakeAll();
    recordCmds(0, 0, 0, 0);
    return;
  }

  recordCmds(fl, fr, rl, rr);
  _fl.drive(fl);
  _fr.drive(fr);
  _rl.drive(rl);
  _rr.drive(rr);
}

void DriveSystem::applyDriveAll(int16_t pwm) {
  applyDrive(pwm, pwm, pwm, pwm);
}

void DriveSystem::brake() {
  brakeAll();
  recordCmds(0, 0, 0, 0);
}

void DriveSystem::coast() {
  coastAll();
  recordCmds(0, 0, 0, 0);
}

void DriveSystem::stop() {
  brakeAll();
  coastAll();
  recordCmds(0, 0, 0, 0);
}

void DriveSystem::enableAll() {
  _fl.enable();
  _fr.enable();
  _rl.enable();
  _rr.enable();
}

void DriveSystem::coastAll() {
  _fl.coast();
  _fr.coast();
  _rl.coast();
  _rr.coast();
}

void DriveSystem::brakeAll() {
  _fl.brake();
  _fr.brake();
  _rl.brake();
  _rr.brake();
}

void DriveSystem::recordCmds(int16_t fl, int16_t fr, int16_t rl, int16_t rr) {
  _lastCmd[0] = fl;
  _lastCmd[1] = fr;
  _lastCmd[2] = rl;
  _lastCmd[3] = rr;
}

void DriveSystem::getMotorPercents(uint8_t out[4]) const {
  for (uint8_t i = 0; i < 4; ++i) {
    uint16_t mag = abs(_lastCmd[i]);
    if (mag > 255) mag = 255;
    out[i] = static_cast<uint8_t>((mag * 100U) / 255U);
  }
}
