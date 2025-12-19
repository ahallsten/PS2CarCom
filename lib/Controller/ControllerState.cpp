#include "ControllerState.h"

bool stateChanged(const ControllerState &current, const ControllerState &previous) {
  return current.buttonWord != previous.buttonWord ||
         current.leftX != previous.leftX || current.leftY != previous.leftY ||
         current.rightX != previous.rightX || current.rightY != previous.rightY;
}

void ButtonEdgeTracker::reset(uint16_t state) {
  _previous = state;
  _initialized = true;
}

void ButtonEdgeTracker::update(uint16_t current,
                               void (*onRise)(uint8_t),
                               void (*onFall)(uint8_t),
                               void (*onState)(uint8_t, bool)) {
  if (!_initialized) {
    _previous = current;
    _initialized = true;
  }

  uint16_t prev = _previous;
  for (uint8_t bit = 0; bit < 16; ++bit) {
    bool currentBit = (current >> bit) & 0x01;
    bool previousBit = (prev >> bit) & 0x01;

    if (currentBit && !previousBit) {
      if (onRise) onRise(bit);
    } else if (!currentBit && previousBit) {
      if (onFall) onFall(bit);
    }

    if (onState) onState(bit, currentBit);
  }

  _previous = current;
}

void printControllerState(const ControllerState &state, Print &out) {
  out.print(F("ButtonWord: "));
  for (int i = 15; i >= 0; --i) {
    out.print(bitRead(state.buttonWord, i));
  }

  out.print(F(" | LX: "));
  out.print(state.leftX);
  out.print(F(" | LY: "));
  out.print(state.leftY);
  out.print(F(" | RX: "));
  out.print(state.rightX);
  out.print(F(" | RY: "));
  out.println(state.rightY);
}

void printDriveDebug(int16_t leftY, int16_t leftX, int16_t rightY, int16_t rightX,
                     bool enabled, bool tankMode, bool parkingBrake, Print &out) {
  out.print(F(" | pwmLY "));
  out.print(leftY);
  out.print(F(" | pwmLX "));
  out.print(leftX);
  out.print(F(" | pwmRY "));
  out.print(rightY);
  out.print(F(" | pwmSTR "));
  out.print(rightX);

  out.print(F(" | EN: "));
  out.print(enabled ? F("ON") : F("OFF"));
  out.print(F(" | T_MD: "));
  out.print(tankMode ? F("ON") : F("OFF"));
  out.print(F(" | p_Brk: "));
  out.println(parkingBrake ? F("ON") : F("OFF"));
}
