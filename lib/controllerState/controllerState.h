#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H

#include <stdint.h>

// Structure representing the state of the PS2 controller.
// It contains a 16‑bit button word and four analog values for the joysticks.
struct ControllerState {
    uint16_t buttonWord;  // bitmask of pressed buttons
    uint8_t leftX;        // left joystick X axis
    uint8_t leftY;        // left joystick Y axis
    uint8_t rightX;       // right joystick X axis
    uint8_t rightY;       // right joystick Y axis
};

// Enumeration of individual button bits used by the PS2 controller.
enum Button {
    SQUARE   = 15,
    CROSS    = 14,
    CIRCLE   = 13,
    TRIANGLE = 12,
    R1       = 11,
    L1       = 10,
    R2       = 9,
    L2       = 8,
    LEFT     = 7,
    RIGHT    = 6,
    DOWN     = 5,
    UP       = 4,
    START    = 3,
    R3       = 2,
    L3       = 1,
    SELECT   = 0,
    UNKNOWN
};

#endif // CONTROLLER_STATE_H