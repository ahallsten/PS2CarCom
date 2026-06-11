#ifndef SoftwarePWM_h
#define SoftwarePWM_h

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

/**
 * @brief Identifies whether a pin lives on the MCU or the MCP23017 expander.
 */
enum class PinSource {
  /** Pin is a direct Arduino/microcontroller pin. */
  MCU_PIN,

  /** Pin is addressed through the MCP23017 GPIO expander. */
  MCP_PIN
};

/**
 * @brief Pin identifier used by software PWM and motor-driver code.
 */
struct PinDef {
  /** Arduino pin number or MCP23017 pin index, depending on source. */
  uint8_t pin;

  /** Location of the pin. */
  PinSource source;
};

/**
 * @brief Runtime state for one SoftwarePWMX output channel.
 */
struct PWMChannelX {
  /** Pin driven by this channel. */
  PinDef pinDef;

  /** Duty cycle, 0..255. */
  uint8_t duty = 0;                 // 0–255

  /** PWM period in microseconds. Defaults to 20 ms, or 50 Hz. */
  unsigned long period_us = 20000UL; // default 50 Hz

  /** Cached high-time duration in microseconds. */
  unsigned long on_time_us = 0;

  /** Last transition time from micros(). */
  unsigned long last_time_us = 0;

  /** Current output state tracked to avoid redundant writes. */
  bool state = false;
};

/**
 * @brief Read-only debug snapshot of one SoftwarePWMX channel.
 */
struct SoftwarePwmSnapshot {
  /** Channel index returned by addChannel(). */
  int8_t channel = -1;

  /** Whether the channel index is valid and currently allocated. */
  bool valid = false;

  /** Pin driven by this channel. */
  PinDef pinDef = { 255, PinSource::MCP_PIN };

  /** Duty cycle currently stored by the scheduler, 0..255. */
  uint8_t duty = 0;

  /** PWM period in microseconds. */
  unsigned long period_us = 0;

  /** Cached high-time duration in microseconds. */
  unsigned long on_time_us = 0;

  /** Current scheduler output state, updated by update(). */
  bool state = false;
};

/**
 * @brief Software PWM scheduler for MCU and MCP23017 pins.
 *
 * This class lets the receiver generate PWM on MCP23017 pins, which do not have
 * hardware PWM. Call update() frequently from loop() so output timing can be
 * maintained.
 */
class SoftwarePWMX {
public:
  /**
   * @brief Allocate a software PWM scheduler.
   *
   * @param maxChannels Maximum number of channels to allocate.
   * @param mcp Optional MCP23017 instance used for channels with MCP_PIN source.
   */
  SoftwarePWMX(uint8_t maxChannels = 8, Adafruit_MCP23X17 *mcp = nullptr);

  /** @brief Release the allocated channel array. */
  ~SoftwarePWMX ();

  /**
   * @brief Add a software PWM output channel.
   *
   * @param pin Output pin definition.
   * @param duty Initial duty cycle, 0..255.
   * @param period_us PWM period in microseconds.
   * @return Channel index, or -1 if no channel slot is available.
   */
  int8_t addChannel(PinDef pin, uint8_t duty = 0, unsigned long period_us = 20000UL);

  /**
   * @brief Remove a PWM channel and drive its pin low.
   *
   * Invalid channel indexes are ignored.
   *
   * @param idx Channel index returned by addChannel().
   */
  void removeChannel(int8_t idx);

  /**
   * @brief Set a channel duty cycle.
   *
   * Invalid or unused channel indexes are ignored.
   *
   * @param idx Channel index returned by addChannel().
   * @param duty Duty cycle, 0..255.
   */
  void setDuty(int8_t idx, uint8_t duty);

  /**
   * @brief Immediately drive a channel low and set its duty cycle to zero.
   *
   * This is used for H-bridge break-before-make direction changes, where the
   * inactive side must be physically low before the opposite side receives PWM.
   *
   * @param idx Channel index returned by addChannel().
   */
  void forceLow(int8_t idx);

  /**
   * @brief Set a channel PWM period.
   *
   * Invalid or unused channel indexes are ignored.
   *
   * @param idx Channel index returned by addChannel().
   * @param period_us New PWM period in microseconds.
   */
  void setPeriod(int8_t idx, unsigned long period_us);

  /**
   * @brief Update all PWM outputs based on micros().
   *
   * Call this as often as possible from loop().
   */
  void update();

  /**
   * @brief Capture the current scheduler state for a channel.
   *
   * This reports the duty cycle the motor layer most recently requested from
   * SoftwarePWMX. The state field is only the scheduler's current HIGH/LOW
   * phase and will change continuously while PWM is running.
   *
   * @param idx Channel index returned by addChannel().
   * @param snapshot Destination populated with channel state.
   * @return true when idx refers to an allocated channel.
   */
  bool getChannelSnapshot(int8_t idx, SoftwarePwmSnapshot &snapshot) const;

  /**
   * @brief Convert an 8-bit duty cycle into high-time microseconds.
   *
   * @param duty Duty cycle, 0..255.
   * @param period_us PWM period in microseconds.
   * @return High-time duration in microseconds.
   */
  static unsigned long dutyToMicros(uint8_t duty, unsigned long period_us);

private:
  /** @brief Route digitalWrite() to the MCU or MCP23017 based on PinDef. */
  void digitalWriteX(PinDef pin, uint8_t val);

  /** @brief Route pinMode() to the MCU or MCP23017 based on PinDef. */
  void pinModeX(PinDef pin, uint8_t mode);

  PWMChannelX *_ch;
  uint8_t _max;
  Adafruit_MCP23X17 *_mcp;
};

#endif
