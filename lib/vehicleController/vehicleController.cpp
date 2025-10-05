#include "VehicleController.h"

VehicleController::VehicleController(MotorDriver &motor)
: motorDriver(motor),
  parking_Brake(false),
  enableToggle(false),
  enableAMotorFwd(false),
  enableAMotorRev(false),
  enableBMotorFwd(false),
  enableBMotorRev(false),
  tankMode(false),
  leftYForward(false),
  rightYForward(false),
  pwmValueLY(0),
  pwmValueRY(0),
  steerPwmValue(0),
  maximumSpeed(0),
  minimumSpeed(25),
  previousButtonWord(0)
{}

void VehicleController::begin() {
    // Initialise the motor driver hardware.
    motorDriver.begin();
}

Button VehicleController::getButtonEnum(uint8_t bit) const {
    switch (bit) {
        case 0:  return SELECT;
        case 1:  return L3;
        case 2:  return R3;
        case 3:  return START;
        case 4:  return UP;
        case 5:  return RIGHT;
        case 6:  return DOWN;
        case 7:  return LEFT;
        case 8:  return L2;
        case 9:  return R2;
        case 10: return L1;
        case 11: return R1;
        case 12: return TRIANGLE;
        case 13: return CIRCLE;
        case 14: return CROSS;
        case 15: return SQUARE;
        default: return UNKNOWN;
    }
}

void VehicleController::handleButtonPress(uint8_t bit) {
    switch (getButtonEnum(bit)) {
        case SELECT:
            parking_Brake = !parking_Brake;
            break;
        case START:
            tankMode = !tankMode;
            break;
        case UP:
            if (maximumSpeed < 250) {
                maximumSpeed += 25;
            } else {
                maximumSpeed = 250;
            }
            break;
        case DOWN:
            if (maximumSpeed > 25) {
                maximumSpeed -= 25;
            } else {
                maximumSpeed = minimumSpeed;
            }
            break;
        case LEFT:
            maximumSpeed = minimumSpeed;
            break;
        case RIGHT:
            maximumSpeed = 250;
            break;
        case L1:
            enableAMotorFwd = true;
            enableAMotorRev = true;
            break;
        case R1:
            enableBMotorFwd = true;
            enableBMotorRev = true;
            break;
        case R2:
            enableToggle = !enableToggle;
            break;
        default:
            // Other buttons may be handled here or ignored.
            break;
    }
}

void VehicleController::detectButtonWordStateChange(uint16_t buttonWord) {
    // Iterate through each bit to check for rising edges.
    for (uint8_t bit = 0; bit < 16; ++bit) {
        bool currentBit = (buttonWord >> bit) & 0x01;
        bool prevBit    = (previousButtonWord >> bit) & 0x01;
        if (!prevBit && currentBit) {
            handleButtonPress(bit);
        }
    }
    // Save the current state for next time.
    previousButtonWord = buttonWord;
}

void VehicleController::update(const ControllerState &current, const ControllerState &prev) {
    // Only recompute outputs if something changed.
    if (current.buttonWord != prev.buttonWord ||
        current.leftX    != prev.leftX    ||
        current.leftY    != prev.leftY    ||
        current.rightX   != prev.rightX   ||
        current.rightY   != prev.rightY) {

        // Handle button changes.
        detectButtonWordStateChange(current.buttonWord);

        // Map left joystick Y axis to PWM for left motor(s).
        if (current.leftY < 110) {
            pwmValueLY = map(current.leftY, 110, 0, 0, 255);
            leftYForward = true;
        } else if (current.leftY > 145) {
            pwmValueLY = map(current.leftY, 145, 255, 0, 255);
            leftYForward = false;
        } else {
            pwmValueLY = 0;
        }

        // Map right joystick Y axis to PWM for right motor in tank mode.
        if (current.rightY < 106) {
            pwmValueRY = map(current.rightY, 106, 0, 0, 255);
            rightYForward = true;
        } else if (current.rightY > 133) {
            pwmValueRY = map(current.rightY, 133, 255, 0, 255);
            rightYForward = false;
        } else {
            pwmValueRY = 0;
        }

        // Map right joystick X axis to steering PWM.
        if (current.rightX > 140) {
            steerPwmValue = map(current.rightX, 135, 255, 0, 255);
        } else if (current.rightX < 110) {
            steerPwmValue = map(current.rightX, 105, 0, 0, 255);
        } else {
            steerPwmValue = 0;
        }

        // Apply parking brake logic.
        if (parking_Brake) {
            pwmValueLY = 0;
            pwmValueRY = 0;
            enableAMotorFwd = false;
            enableBMotorFwd = false;
            enableAMotorRev = false;
            enableBMotorRev = false;
        } else {
            // Constrain PWM values by maximum speed.
            pwmValueLY = map(pwmValueLY, 0, 255, 0, maximumSpeed);
            pwmValueRY = map(pwmValueRY, 0, 255, 0, maximumSpeed);
        }

        // Update motor enable flags based on the enable toggle and direction.
        if (enableToggle) {
            if (leftYForward) {
                enableAMotorFwd = true;
                enableBMotorFwd = true;
                enableAMotorRev = false;
                enableBMotorRev = false;
            } else {
                enableAMotorFwd = false;
                enableBMotorFwd = false;
                enableAMotorRev = true;
                enableBMotorRev = true;
            }
        } else {
            enableAMotorFwd = false;
            enableBMotorFwd = false;
            enableAMotorRev = false;
            enableBMotorRev = false;
        }

        // Apply tank mode overrides for independent left/right control.
        if (tankMode) {
            if (current.leftY < 110) {
                enableAMotorFwd = true;
                enableAMotorRev = false;
            } else if (current.leftY > 145) {
                enableAMotorFwd = false;
                enableAMotorRev = true;
            }
            if (current.rightY < 106) {
                enableBMotorFwd = true;
                enableBMotorRev = false;
            } else if (current.rightY > 133) {
                enableBMotorFwd = false;
                enableBMotorRev = true;
            }
        }

        // Send the computed values to the motor driver.
        motorDriver.apply(tankMode, pwmValueLY, pwmValueRY, steerPwmValue,
                          enableAMotorFwd, enableAMotorRev,
                          enableBMotorFwd, enableBMotorRev);
    }
}