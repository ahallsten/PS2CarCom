#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

// Only build this class when the TRANSMITTER macro is defined.
#ifdef TRANSMITTER

#include <Arduino.h>
#include <PsxControllerBitBang.h>
#include "ControllerState.h"

// Default pin assignments for the PS2 controller.
static const byte PIN_PS2_ATT = 10;
static const byte PIN_PS2_CMD = 11;
static const byte PIN_PS2_DAT = 12;
static const byte PIN_PS2_CLK = 13;

// Class encapsulating a PS2 controller connected via the bit‑banged protocol.
// It handles initialisation and reading of the controller's state.
class Ps2Controller {
public:
    Ps2Controller();

    // Initialise the PS2 controller. Returns true on success.
    bool begin();

    // Reads the current controller state into the provided ControllerState object.
    // Returns true if the state was successfully read.
    bool readState(ControllerState &state);

    // Returns true if the controller is currently connected.
    bool isConnected() const;

private:
    PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;
    bool haveController;
};

#endif // TRANSMITTER

#endif // PS2_CONTROLLER_H