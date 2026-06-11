/**
 * @file BTS7960.h
 * @brief Driver wrapper for one BTS7960 H-bridge motor controller.
 */
#ifndef BTS7960_h
#define BTS7960_h

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include "Pca9685Pwm.h"

enum class PinSource {
  /** Direct Arduino/microcontroller pin. */
  MCU_PIN,

  /** MCP23017 GPIO expander pin index. */
  MCP_PIN
};

struct PinDef {
  /** Arduino pin number or MCP23017 pin index, depending on source. */
  uint8_t pin;

  /** Pin location. */
  PinSource source;
};

/**
<<<<<<< Updated upstream
 * @brief Debug snapshot of one BTS7960 driver's PWM-side state.
 */
struct Bts7960PwmSnapshot {
  /** Scheduler state for the RPWM input. */
  SoftwarePwmSnapshot rpwm;

  /** Scheduler state for the LPWM input. */
  SoftwarePwmSnapshot lpwm;

  /** Last nonzero drive direction selected by this wrapper. */
  int8_t direction = 0;
};

/**
 * @brief Controls one BTS7960 motor driver through PWM, enable, and sense pins.
=======
 * @brief Controls one BTS7960 motor driver through PCA9685 PWM plus enable/sense pins.
>>>>>>> Stashed changes
 *
 * RPWM/LPWM are PCA9685 channel numbers. Enable pins may be on the MCU or
 * MCP23017. Current-sense pins are analog-read only when configured as MCU pins.
 */
class BTS7960 {
public:
  BTS7960(Adafruit_MCP23X17 *mcp,
          Pca9685Pwm *pwm,
          uint8_t rPwmChannel,
          uint8_t lPwmChannel,
          PinDef L_EN,
          PinDef R_EN,
          PinDef L_IS,
          PinDef R_IS);

  void begin();
  void drive(int16_t pwm);
  void coast();
  void brake();
  void ccwBrake();
  void cwBrake();
  void enable();
  void disable();
  void stop();

<<<<<<< Updated upstream
  /** @brief Capture the current PWM scheduler state for both direction pins. */
  void getPwmSnapshot(Bts7960PwmSnapshot &snapshot) const;
=======
  /** Read raw L_IS and R_IS ADC values; non-MCU sense pins report zero. */
  void readCurrentSense(uint16_t &lisOut, uint16_t &risOut) const;
>>>>>>> Stashed changes

private:
  Adafruit_MCP23X17 *_mcp;
  Pca9685Pwm *_pwm;

  uint8_t _rPwmChannel;
  uint8_t _lPwmChannel;
  PinDef _L_EN, _R_EN;
  PinDef _L_IS, _R_IS;
  int8_t _lastDir = 0;

  void pinModeX(PinDef pin, uint8_t mode);
  void digitalWriteX(PinDef pin, uint8_t value);
  uint16_t analogReadX(PinDef pin) const;
};

#endif
