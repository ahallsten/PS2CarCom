// #ifndef MY_LIBRARY_H
// #define MY_LIBRARY_H

// #include <Arduino.h>
// #include <RH_RF95.h>

// // Define structs and enums
// struct ControllerState {
//     uint16_t buttonWord;
//     uint8_t leftX;
//     uint8_t leftY;
//     uint8_t rightX;
//     uint8_t rightY;
// };

// enum Button {
//     SELECT,
//     START,
//     UP,
//     DOWN,
//     LEFT,
//     RIGHT,
//     TRIANGLE,
//     CIRCLE,
//     CROSS,
//     SQUARE,
//     L1,
//     R1,
//     L2,
//     R2,
//     L3,
//     R3,
//     UNKNOWN
// };

// // Declare functions
// void setupReceiver(RH_RF95 &rfm);
// void receivePacket(RH_RF95 &rfm, ControllerState &currentState);
// void controlsDecision(const ControllerState &currentState, ControllerState &previousState);
// void handleButtonPress(const String &buttonName);
// Button getButtonEnum(const String &buttonName);

// #endif // MY_LIBRARY_H