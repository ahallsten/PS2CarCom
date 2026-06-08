/**
 * @file BTS7960.h
 * @brief Driver wrapper for one BTS7960 H-bridge motor controller.
 */
#ifndef BTS7960_h
#define BTS7960_h

#include "Arduino.h"
#include "SoftwarePWMX.h"
#include <Adafruit_MCP23X17.h>

/**
 * @brief Controls one BTS7960 motor driver through PWM, enable, and sense pins.
 *
 * Pins may live directly on the microcontroller or on the MCP23017 GPIO
 * expander. PWM output is delegated to SoftwarePWMX so MCP pins can be used for
 * BTS7960 RPWM/LPWM inputs.
 */
class BTS7960 {
public:
  /**
   * @brief Create a BTS7960 wrapper.
   *
   * @param mcp MCP23017 object used when any PinDef has PinSource::MCP_PIN.
   * @param pwmx Software PWM provider used for RPWM and LPWM pins.
   * @param RPWM Right/PWM input pin for one drive direction.
   * @param LPWM Left/PWM input pin for the opposite drive direction.
   * @param L_EN Left-side enable pin.
   * @param R_EN Right-side enable pin.
   * @param L_IS Left-side current-sense/status input pin.
   * @param R_IS Right-side current-sense/status input pin.
   */
  BTS7960(Adafruit_MCP23X17 *mcp,
          SoftwarePWMX *pwmx,
          PinDef RPWM, PinDef LPWM,
          PinDef L_EN, PinDef R_EN,
          PinDef L_IS, PinDef R_IS);

  /** @brief Configure all pins, allocate PWM channels, and leave the driver coasting. */
  void begin();

  /**
   * @brief Command signed motor PWM.
   *
   * Positive values drive RPWM, negative values drive LPWM, and zero calls
   * brake(). Values are constrained to -255..255.
   *
   * @param pwm Signed speed and direction command.
   */
  void drive(int16_t pwm);

  /** @brief Disable both BTS7960 enable pins so the motor coasts. */
  void coast();

  /** @brief Set both PWM channels low while leaving enable state unchanged. */
  void brake();

  /** @brief Enable only the counter-clockwise braking side. */
  void ccwBrake();

  /** @brief Enable only the clockwise braking side. */
  void cwBrake();

  /** @brief Enable both BTS7960 sides. */
  void enable();

  /** @brief Disable the driver by coasting. */
  void disable();

  /** @brief Brake, then coast the driver. */
  void stop();

private:
  Adafruit_MCP23X17 *_mcp;
  SoftwarePWMX *_pwmx;

  PinDef _RPWM, _LPWM;
  PinDef _L_EN, _R_EN;
  PinDef _L_IS, _R_IS;
  int8_t _rPwmCh = -1;
  int8_t _lPwmCh = -1;
  int8_t _lastDir = 0;

  /** @brief Route pinMode() to either the MCU or MCP23017 based on the PinDef. */
  void pinModeX(PinDef pin, uint8_t mode);

  /** @brief Route digitalWrite() to either the MCU or MCP23017 based on the PinDef. */
  void digitalWriteX(PinDef pin, uint8_t value);
};
#endif
