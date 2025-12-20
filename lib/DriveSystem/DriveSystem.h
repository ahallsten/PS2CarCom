#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "BTS7960.h"
#include "SoftwarePWMX.h"

struct MotorPins {
  PinDef rpwm;
  PinDef lpwm;
  PinDef len;
  PinDef ren;
  PinDef lis;
  PinDef ris;
};

class DriveSystem {
public:
  DriveSystem(Adafruit_MCP23X17 &mcp,
              SoftwarePWMX &pwmx,
              const MotorPins &fl,
              const MotorPins &fr,
              const MotorPins &rl,
              const MotorPins &rr);

  void begin();
  void setEnabled(bool enabled);
  void setParkingBrake(bool enabled);
  void applyDrive(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
  void applyDriveAll(int16_t pwm);
  void brake();
  void coast();
  void stop();
  void getMotorPercents(uint8_t out[4]) const;

private:
  void enableAll();
  void coastAll();
  void brakeAll();
  void recordCmds(int16_t fl, int16_t fr, int16_t rl, int16_t rr);

  BTS7960 _fl;
  BTS7960 _fr;
  BTS7960 _rl;
  BTS7960 _rr;

  bool _enabled = false;
  bool _parkingBrake = false;
  int16_t _lastCmd[4] = {0, 0, 0, 0};
};

#endif
