g// #include "MyLibrary.h"

// // Setup the receiver
// void setupReceiver(RH_RF95 &rfm) {
//     pinMode(RFM95_RST, OUTPUT);
//     digitalWrite(RFM95_RST, HIGH);

//     if (!rfm.init()) {
//         Serial.println("RFM95 initialization failed");
//         while (1);
//     }
//     rfm.setFrequency(915.0);
//     Serial.println("RFM95 initialized: RECEIVER");
// }

// // Receive a packet and update the current state
// void receivePacket(RH_RF95 &rfm, ControllerState &currentState) {
//     if (rfm.available()) {
//         uint8_t buffer[sizeof(ControllerState)];
//         uint8_t len = sizeof(buffer);

//         // Receive the packet
//         rfm.recv(buffer, &len);

//         // Deserialize into a struct
//         memcpy(&currentState, buffer, sizeof(ControllerState));

//         // Debugging output
//         Serial.print(F("ButtonWord: "));
//         for (int i = 15; i >= 0; i--) {
//             Serial.print((currentState.buttonWord & (1 << i)) ? '1' : '0');
//         }
//         Serial.print(F(" (HEX: "));
//         Serial.print(currentState.buttonWord, HEX);
//         Serial.println(F(")"));
//     }
// }

// // Handle button presses
// void handleButtonPress(const String &buttonName) {
//     if (buttonName == "Select") {
//         Serial.println(F("Select button pressed!"));
//     } else if (buttonName == "Start") {
//         Serial.println(F("Start button pressed!"));
//     }
//     // Add other button cases here...
// }

// // Map button names to enums
// Button getButtonEnum(const String &buttonName) {
//     if (buttonName == "Select") return SELECT;
//     if (buttonName == "Start") return START;
//     if (buttonName == "Up") return UP;
//     if (buttonName == "Down") return DOWN;
//     if (buttonName == "Left") return LEFT;
//     if (buttonName == "Right") return RIGHT;
//     if (buttonName == "Triangle") return TRIANGLE;
//     if (buttonName == "Circle") return CIRCLE;
//     if (buttonName == "Cross") return CROSS;
//     if (buttonName == "Square") return SQUARE;
//     if (buttonName == "L1") return L1;
//     if (buttonName == "R1") return R1;
//     if (buttonName == "L2") return L2;
//     if (buttonName == "R2") return R2;
//     if (buttonName == "L3") return L3;
//     if (buttonName == "R3") return R3;
//     return UNKNOWN;
// }

// // Make control decisions based on the current state
// void controlsDecision(const ControllerState &currentState, ControllerState &previousState) {
//     if (currentState.buttonWord != previousState.buttonWord) {
//         previousState = currentState;

//         // Example: Handle button presses
//         String buttonName = "Start"; // Replace with actual button name logic
//         handleButtonPress(buttonName);
//     }
// }