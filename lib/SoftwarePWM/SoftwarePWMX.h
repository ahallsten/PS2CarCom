#ifndef SoftwarePWM_h
#define SoftwarePWM_h

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

// Represents where a pin lives (MCU vs. MCP)
enum class PinSource {
  MCU_PIN,
  MCP_PIN
};

struct PinDef {
  uint8_t pin;
  PinSource source;
};

struct PWMChannelX {
  PinDef pinDef;
  uint8_t duty = 0;                 // 0–255
  unsigned long period_us = 20000UL; // default 50 Hz
  unsigned long on_time_us = 0;
  unsigned long last_time_us = 0;
  bool state = false;
};

class SoftwarePWMX {
public:
  SoftwarePWMX(uint8_t maxChannels = 8, Adafruit_MCP23X17 *mcp = nullptr);
  ~SoftwarePWMX ();

  int8_t addChannel(PinDef pin, uint8_t duty = 0, unsigned long period_us = 20000UL);
  void removeChannel(int8_t idx);
  void setDuty(int8_t idx, uint8_t duty);
  void setPeriod(int8_t idx, unsigned long period_us);
  void update();

  // Static helper
  static unsigned long dutyToMicros(uint8_t duty, unsigned long period_us);

private:
  void digitalWriteX(PinDef pin, uint8_t val);
  void pinModeX(PinDef pin, uint8_t mode);

  PWMChannelX *_ch;
  uint8_t _max;
  Adafruit_MCP23X17 *_mcp;
};

#endif
