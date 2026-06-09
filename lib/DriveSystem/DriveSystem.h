#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "BTS7960.h"
#include "SoftwarePWMX.h"

/**
 * @brief Pin bundle for one BTS7960-controlled motor.
 *
 * Each field is a PinDef so the receiver can mix MCP23017 pins and direct MCU
 * pins in one motor configuration.
 */
struct MotorPins {
  /** PWM pin for the BTS7960 forward/right input. */
  PinDef rpwm;

  /** PWM pin for the BTS7960 reverse/left input. */
  PinDef lpwm;

  /** Enable pin paired with LPWM. */
  PinDef len;

  /** Enable pin paired with RPWM. */
  PinDef ren;

  /** Sense/status input paired with LPWM. */
  PinDef lis;

  /** Sense/status input paired with RPWM. */
  PinDef ris;
};

/**
 * @brief Coordinates the four vehicle motor drivers.
 *
 * DriveSystem owns four BTS7960 wrappers and applies receiver-level behavior
 * such as enable/disable, parking brake, braking, coasting, and motor percent
 * telemetry.
 */
class DriveSystem {
public:
  /**
   * @brief Create a four-motor drive system.
   *
   * @param mcp MCP23017 instance used by motor pins configured as MCP_PIN.
   * @param pwmx Software PWM provider for BTS7960 PWM channels.
   * @param fl Front-left motor pins.
   * @param fr Front-right motor pins.
   * @param rl Rear-left motor pins.
   * @param rr Rear-right motor pins.
   */
  DriveSystem(Adafruit_MCP23X17 &mcp,
              SoftwarePWMX &pwmx,
              const MotorPins &fl,
              const MotorPins &fr,
              const MotorPins &rl,
              const MotorPins &rr);

  /** @brief Initialize all four motor drivers. */
  void begin();

  /**
   * @brief Enable or disable all drivers.
   *
   * Disabling puts all drivers into coast mode.
   *
   * @param enabled true to enable the motor drivers, false to coast them.
   */
  void setEnabled(bool enabled);

  /**
   * @brief Enable or disable parking brake behavior.
   *
   * Enabling the parking brake immediately brakes all motors. While the parking
   * brake is active, applyDrive() records and applies zero commands.
   *
   * @param enabled true to activate parking brake behavior.
   */
  void setParkingBrake(bool enabled);

  /**
   * @brief Apply signed PWM commands to all four motors.
   *
   * @param fl Front-left signed PWM command.
   * @param fr Front-right signed PWM command.
   * @param rl Rear-left signed PWM command.
   * @param rr Rear-right signed PWM command.
   */
  void applyDrive(int16_t fl, int16_t fr, int16_t rl, int16_t rr);

  /**
   * @brief Apply the same signed PWM command to all four motors.
   *
   * @param pwm Signed PWM command for every motor.
   */
  void applyDriveAll(int16_t pwm);

  /** @brief Brake all motors and record zero motor commands. */
  void brake();

  /** @brief Coast all motors and record zero motor commands. */
  void coast();

  /** @brief Brake and then coast all motors, recording zero motor commands. */
  void stop();

  /**
   * @brief Return the absolute command percentage for each motor.
   *
   * Percentages are derived from the last recorded command magnitudes in
   * front-left, front-right, rear-left, rear-right order.
   *
   * @param out Four-byte output array populated with 0..100 percentages.
   */
  void getMotorPercents(uint8_t out[4]) const;

  /**
   * @brief Return the latest signed commands recorded for FL, FR, RL, RR.
   *
   * These are the commands that survived receiver-level enable and parking
   * brake checks, not merely the raw stick-mapping values.
   *
   * @param out Four signed command values in front-left, front-right,
   * rear-left, rear-right order.
   */
  void getLastCommands(int16_t out[4]) const;

  /**
   * @brief Capture PWM scheduler state for FL, FR, RL, RR.
   *
   * @param out Four motor PWM snapshots in front-left, front-right,
   * rear-left, rear-right order.
   */
  void getPwmSnapshots(Bts7960PwmSnapshot out[4]) const;

private:
  /** @brief Enable all four BTS7960 drivers. */
  void enableAll();

  /** @brief Coast all four BTS7960 drivers. */
  void coastAll();

  /** @brief Brake all four BTS7960 drivers. */
  void brakeAll();

  /** @brief Save the most recent signed motor commands for status telemetry. */
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
