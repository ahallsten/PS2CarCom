#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "BTS7960.h"
#include "Pca9685Pwm.h"
#include "VehicleLayout.h"

struct MotorPins {
  /** PCA9685 channel for the BTS7960 RPWM direction input. */
  uint8_t rpwmChannel;

  /** PCA9685 channel for the BTS7960 LPWM direction input. */
  uint8_t lpwmChannel;

  /** Enable pin paired with LPWM. */
  PinDef len;

  /** Enable pin paired with RPWM. */
  PinDef ren;

  /** Current-sense pin paired with LPWM. */
  PinDef lis;

  /** Current-sense pin paired with RPWM. */
  PinDef ris;
};

/**
 * @brief Coordinates the four drive motors plus steering BTS7960 channel.
 */
class DriveSystem {
public:
  DriveSystem(Adafruit_MCP23X17 &mcp,
              Pca9685Pwm &pwm,
              const MotorPins &fl,
              const MotorPins &fr,
              const MotorPins &rl,
              const MotorPins &rr,
              const MotorPins &steer);

  void begin();
  void setEnabled(bool enabled);
  void setParkingBrake(bool enabled);

  /** Apply signed PWM commands in FL, FR, RL, RR, steering order. */
  void applyDrive(int16_t fl, int16_t fr, int16_t rl, int16_t rr, int16_t steer);
  void applyDriveAll(int16_t pwm);
  void brake();
  void coast();
  void stop();

  /** Return last allowed signed commands in VehicleMotorIndex order. */
  void getMotorCommands(int16_t out[VEHICLE_MOTOR_COUNT]) const;

  /** Read raw current-sense values in VehicleCurrentSenseIndex order. */
  void readCurrentSense(uint16_t out[VEHICLE_CURRENT_SENSE_COUNT]) const;

  /**
   * @brief Capture PCA9685 PWM state in VehicleMotorIndex order.
   *
   * @param out Motor PWM snapshots in FL, FR, RL, RR, steering order.
   */
  void getPwmSnapshots(Bts7960PwmSnapshot out[VEHICLE_MOTOR_COUNT]) const;

private:
  void enableAll();
  void coastAll();
  void brakeAll();
  void recordCmds(int16_t fl, int16_t fr, int16_t rl, int16_t rr, int16_t steer);

  BTS7960 _fl;
  BTS7960 _fr;
  BTS7960 _rl;
  BTS7960 _rr;
  BTS7960 _steer;

  bool _enabled = false;
  bool _parkingBrake = false;
  int16_t _lastCmd[VEHICLE_MOTOR_COUNT] = {0, 0, 0, 0, 0};
};

#endif
