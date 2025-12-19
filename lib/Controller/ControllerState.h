#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H

#include <Arduino.h>

struct ControllerState {
  uint16_t buttonWord;
  uint8_t leftX;
  uint8_t leftY;
  uint8_t rightX;
  uint8_t rightY;
};

bool stateChanged(const ControllerState &current, const ControllerState &previous);

class ButtonEdgeTracker {
public:
  void reset(uint16_t state = 0);
  void update(uint16_t current,
              void (*onRise)(uint8_t),
              void (*onFall)(uint8_t),
              void (*onState)(uint8_t, bool));

private:
  uint16_t _previous = 0;
  bool _initialized = false;
};

void printControllerState(const ControllerState &state, Print &out);
void printDriveDebug(int16_t leftY, int16_t leftX, int16_t rightY, int16_t rightX,
                     bool enabled, bool tankMode, bool parkingBrake, Print &out);

#endif
