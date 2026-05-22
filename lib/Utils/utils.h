#ifndef utils_h
#define utils_h

#include <Arduino.h>

/**
 * @brief Legacy copy of the controller state shape used by older debug helpers.
 *
 * The active firmware uses lib/Controller/ControllerState.h instead. Keep this
 * definition in sync if these legacy helpers are used again.
 */
struct ControllerState {
    /** Packed 16-bit button state. */
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

/** @brief Controller state expected to be defined by the sketch using utils.cpp. */
extern ControllerState currentState;

/** @brief Left-stick Y PWM/debug value expected from the sketch. */
extern int16_t leftYPWM;

/** @brief Left-stick X PWM/debug value expected from the sketch. */
extern int16_t leftXPWM;

/** @brief Right-stick Y PWM/debug value expected from the sketch. */
extern int16_t rightYPWM;

/** @brief Right-stick X steering/debug value expected from the sketch. */
extern int16_t rightXPWM;

/** @brief Legacy motor-A forward enable debug flag expected from the sketch. */
extern bool enableAMotorFwd;

/** @brief Legacy motor-A reverse enable debug flag expected from the sketch. */
extern bool enableAMotorRev;

/** @brief Legacy motor-B forward enable debug flag expected from the sketch. */
extern bool enableBMotorFwd;

/** @brief Legacy motor-B reverse enable debug flag expected from the sketch. */
extern bool enableBMotorRev;

/** @brief Legacy overall enable debug flag expected from the sketch. */
extern bool enableToggle;

/** @brief Tank-drive mode debug flag expected from the sketch. */
extern bool tankMode;

/** @brief Parking-brake debug flag expected from the sketch. */
extern bool parking_Brake;

/**
 * @brief Print the legacy global controller state to Serial.
 */
void printControllerStruct();

/**
 * @brief Print legacy global PWM/control variables to Serial.
 */
void printControlVariables();
#endif
