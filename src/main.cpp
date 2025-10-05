#include <Arduino.h>
#include "ControllerState.h"
#include "RadioLink.h"

// Define the role of this device at compile time.  In PlatformIO this can
// be set via build flags (e.g. -DTRANSMITTER or -DRECEIVER).  If neither
// macro is defined the sketch will not compile.
// #define TRANSMITTER
// #define RECEIVER

// Pin assignments for the RFM95 radio.  These match the original sketch.
#define RFM95_CS  8
#define RFM95_RST 4
#define RFM95_INT 7

// Radio frequency for the RFM95 module.
#define RF95_FREQ 915.0

#ifdef TRANSMITTER
#include "Ps2Controller.h"
#else
#include "MotorDriver.h"
#include "VehicleController.h"
#endif

// Global radio object used in both transmitter and receiver roles.
RadioLink radio(RFM95_CS, RFM95_INT, RFM95_RST);

// State objects used to track the current and previous controller values.
static ControllerState currentState = {0, 0, 0, 0, 0};
static ControllerState previousState = {0, 0, 0, 0, 0};

#ifdef TRANSMITTER
// Transmitter-specific objects
static Ps2Controller ps2Controller;
#else
// Receiver-specific objects
static MotorDriver motorDriver;
static VehicleController vehicleController(motorDriver);
#endif

void setup() {
    Serial.begin(115200);

    // Initialise the radio module.  Abort if it fails.
    if (!radio.init(RF95_FREQ)) {
        Serial.println(F("RFM95 initialization failed"));
        while (1) {
            // Stay here forever if the radio fails to initialise
        }
    }

    // Depending on the role, initialise additional hardware and state.
#ifdef TRANSMITTER
    if (!ps2Controller.begin()) {
        Serial.println(F("PS2 controller not detected"));
    } else {
        Serial.println(F("RFM95 initialized: TRANSMITTER"));
    }
#else
    motorDriver.begin();
    vehicleController.begin();
    Serial.println(F("RFM95 initialized: RECEIVER"));
#endif
}

void loop() {
#ifdef TRANSMITTER
    // Read the controller state.  If a new state is available, send it.
    if (ps2Controller.readState(currentState)) {
        // Only send when the state has changed to conserve bandwidth.
        if (memcmp(&currentState, &previousState, sizeof(ControllerState)) != 0) {
            radio.sendPacket(&currentState, sizeof(ControllerState));
            previousState = currentState;
        }
    }
    // Limit the loop rate to approximately 140 Hz.
    delay(1000 / 140);
#else
    // In receiver mode, check if a packet is available and process it.
    if (radio.available()) {
        uint8_t len = sizeof(ControllerState);
        if (radio.recvPacket(&currentState, &len)) {
            // Update the vehicle logic with the new controller state.
            vehicleController.update(currentState, previousState);
            previousState = currentState;
        }
    }
#endif
}