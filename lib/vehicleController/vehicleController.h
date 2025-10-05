#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <Arduino.h>
#include "MotorDriver.h"
#include "ControllerState.h"

// VehicleController handles the high‑level control logic for the remote control car.
// It interprets joystick positions and button presses into motor PWM values and steering angles.
class VehicleController {
public:
    // Construct a VehicleController that outputs to the provided MotorDriver.
    explicit VehicleController(MotorDriver &motorDriver);

    // Initialise the vehicle controller and underlying motor driver.
    void begin();

    // Update the vehicle control logic given the current and previous controller states.
    // This should be called whenever a new ControllerState is received.
    void update(const ControllerState &currentState, const ControllerState &prevState);

    // Accessors for some internal flags, if needed externally.
    bool isParkingBrake() const { return parking_Brake; }
    bool isEnableToggle() const { return enableToggle; }

private:
    // Reference to the MotorDriver used for hardware output.
    MotorDriver &motorDriver;

    // Internal state variables mirroring those in the original sketch.
    bool parking_Brake;
    bool enableToggle;
    bool enableAMotorFwd;
    bool enableAMotorRev;
    bool enableBMotorFwd;
    bool enableBMotorRev;
    bool tankMode;
    bool leftYForward;
    bool rightYForward;
    byte pwmValueLY;
    byte pwmValueRY;
    byte steerPwmValue;
    int maximumSpeed;
    int minimumSpeed;
    // Track the previous button word to detect rising edges.
    uint16_t previousButtonWord;

    // Translate a bit index into a Button enum.
    Button getButtonEnum(uint8_t bit) const;

    // Handle the press of a specific button bit.
    void handleButtonPress(uint8_t bit);

    // Detect changes in the button word and dispatch events to handleButtonPress.
    void detectButtonWordStateChange(uint16_t buttonWord);
};

#endif // VEHICLE_CONTROLLER_H