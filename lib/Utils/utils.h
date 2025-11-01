#ifndef utils_h
#define utils_h

#include <Arduino.h>

// Forward declarations of structs and global variables
// (so utils.cpp can access them — these should already exist in your main program)
struct ControllerState {
    uint16_t buttonWord;
    uint8_t leftX;
    uint8_t leftY;
    uint8_t rightX;
    uint8_t rightY;
};

// These variables are defined in your main file (or elsewhere)
extern ControllerState currentState;
extern int16_t leftYPWM;
extern int16_t leftXPWM;
extern int16_t rightYPWM;
extern int16_t rightXPWM;
extern bool enableAMotorFwd;
extern bool enableAMotorRev;
extern bool enableBMotorFwd;
extern bool enableBMotorRev;
extern bool enableToggle;
extern bool tankMode;
extern bool parking_Brake;

// ---------- Function declarations ----------
void printControllerStruct();
void printControlVariables();
#endif