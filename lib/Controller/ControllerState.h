#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H

#include <Arduino.h>

/**
 * @brief Snapshot of the PS2 controller state sent over the radio link.
 *
 * The transmitter fills this from PsxNewLib. The receiver decodes it from the
 * control packet and uses it for button edge detection, drive commands, and
 * steering commands.
 */
struct ControllerState {
  /** Packed 16-bit PS2 button state. Button bit meanings are interpreted by the receiver. */
  uint16_t buttonWord;

  /** Left analog stick X axis, normally 0..255. */
  uint8_t leftX;

  /** Left analog stick Y axis, normally 0..255. */
  uint8_t leftY;

  /** Right analog stick X axis, normally 0..255. */
  uint8_t rightX;

  /** Right analog stick Y axis, normally 0..255. */
  uint8_t rightY;
};

/**
 * @brief Compare two controller snapshots.
 *
 * @param current New controller state.
 * @param previous Previous controller state.
 * @return true if any button or axis value differs.
 */
bool stateChanged(const ControllerState &current, const ControllerState &previous);

/**
 * @brief Tracks rising and falling edges in a 16-bit button word.
 *
 * The receiver uses this to make button actions toggle only once per press
 * instead of repeatedly while the button is held.
 */
class ButtonEdgeTracker {
public:
  /**
   * @brief Set the current baseline button state.
   *
   * @param state Button word to treat as the previous state for future updates.
   */
  void reset(uint16_t state = 0);

  /**
   * @brief Compare a new button word against the previous word and invoke callbacks.
   *
   * Null callbacks are allowed. When provided, onRise is called for each bit that
   * changed from 0 to 1, onFall is called for each bit that changed from 1 to 0,
   * and onState is called for every bit with its current value.
   *
   * @param current Current 16-bit button word.
   * @param onRise Optional callback for button rising edges.
   * @param onFall Optional callback for button falling edges.
   * @param onState Optional callback for every button state.
   */
  void update(uint16_t current,
              void (*onRise)(uint8_t),
              void (*onFall)(uint8_t),
              void (*onState)(uint8_t, bool));

private:
  uint16_t _previous = 0;
  bool _initialized = false;
};

/**
 * @brief Print a standardized log-line prefix shared by both firmware roles.
 *
 * Emits "[<millis>] <role> <tag> seq=<seq> " with no trailing newline, so the
 * caller can append message-specific key=value fields. The millis() timestamp
 * and seq let transmitter and receiver logs be correlated against each other.
 *
 * @param out Destination stream, usually Serial.
 * @param role Short role token, e.g. F("TX") or F("RX").
 * @param tag Short message-kind token, e.g. F("CTRL"), F("DRV"), F("ACK").
 * @param seq Message ID (control sequence number) this line refers to.
 */
void printLogPrefix(Print &out, const __FlashStringHelper *role,
                    const __FlashStringHelper *tag, uint8_t seq);

/**
 * @brief Print a controller snapshot to a Print-compatible stream.
 *
 * Emits compact "btn=<16bit> LX=.. LY=.. RX=.. RY=.." with a trailing newline.
 *
 * @param state State to print.
 * @param out Destination stream, usually Serial.
 */
void printControllerState(const ControllerState &state, Print &out);

/**
 * @brief Print receiver drive-mapping debug values to a Print-compatible stream.
 *
 * @param leftY Mapped left-stick Y command.
 * @param leftX Mapped left-stick X command.
 * @param rightY Mapped right-stick Y command.
 * @param rightX Mapped right-stick X/steering command.
 * @param enabled Whether drive output is currently enabled.
 * @param tankMode Whether tank-drive mode is active.
 * @param parkingBrake Whether parking brake mode is active.
 * @param out Destination stream, usually Serial.
 */
void printDriveDebug(int16_t leftY, int16_t leftX, int16_t rightY, int16_t rightX,
                     bool enabled, bool tankMode, bool parkingBrake, Print &out);

#endif
