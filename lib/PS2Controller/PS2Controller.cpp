#include "Ps2Controller.h"

#ifdef TRANSMITTER

Ps2Controller::Ps2Controller()
: psx(),
  haveController(false)
{}

bool Ps2Controller::begin() {
    if (!psx.begin()) {
        return false;
    }
    // A short delay is recommended after the PS2 controller begins.
    delay(300);
    if (!psx.enterConfigMode()) {
        return false;
    }
    // Attempt to enable analog sticks and buttons.
    psx.enableAnalogSticks();
    psx.enableAnalogButtons();
    psx.exitConfigMode();
    haveController = true;
    return true;
}

bool Ps2Controller::readState(ControllerState &state) {
    // If the controller is not attached, simply return false.
    if (!haveController) {
        return false;
    }
    // Attempt to read the controller.  If this fails, mark it as disconnected.
    if (!psx.read()) {
        haveController = false;
        return false;
    }
    // Populate the controller state structure with the latest values.
    state.buttonWord = psx.getButtonWord();
    psx.getLeftAnalog(state.leftX, state.leftY);
    psx.getRightAnalog(state.rightX, state.rightY);
    return true;
}

bool Ps2Controller::isConnected() const {
    return haveController;
}

#endif // TRANSMITTER