/*
  BTS7960.h - Library to control the BTS7960 motor driver
*/
#ifndef BTS7960_h
#define BTS7960_h

#include "Arduino.h"
#include <Adafruit_MCP23X17.h>

enum PinSource {
  MCU_PIN,
  MCP_PIN
};

struct PinDef {
  uint8_t pin;
  PinSource source;
};

class BTS7960 {
public:
  BTS7960(Adafruit_MCP23X17 *mcp,
          PinDef RPWM, PinDef LPWM,
          PinDef L_EN, PinDef R_EN,
          PinDef L_IS, PinDef R_IS);

  void begin();
  void drive(int16_t pwm);
  void coast();
  void brake();
  void ccwBrake();
  void cwBrake();
  void enable();
  void disable();
  void stop();

private:
  Adafruit_MCP23X17 *_mcp;

  PinDef _RPWM, _LPWM;
  PinDef _L_EN, _R_EN;
  PinDef _L_IS, _R_IS;

  void pinModeX(PinDef pin, uint8_t mode);
  void digitalWriteX(PinDef pin, uint8_t value);
  int analogWriteX(PinDef pin, uint8_t value);
};
#endif