#ifndef RADIO_LINK_H
#define RADIO_LINK_H

#include <Arduino.h>
#include <RH_RF95.h>

// Forward declaration of ControllerState for convenience, not strictly needed here
#include "ControllerState.h"

// RadioLink encapsulates the Adafruit RFM95/RFM9x radio functionality.
// It provides methods to initialise the radio at a given frequency and to
// send/receive opaque packets of arbitrary length.
class RadioLink {
public:
    // Construct a RadioLink object with the given chip select, interrupt and reset pins.
    RadioLink(uint8_t csPin, uint8_t intPin, uint8_t rstPin);

    // Initialise the radio.  Returns true on success.
    bool init(float freq);

    // Send a packet of data.  The packet contents are not interpreted by this class.
    // Returns true if the packet was successfully queued for transmission.
    bool sendPacket(const void* data, uint8_t len);

    // Receive a packet.  If a packet is available it will be copied into the provided buffer and the length updated.
    // Returns true if a packet was received.
    bool recvPacket(void* buffer, uint8_t* len);

    // Returns true if a packet is available to be read.
    bool available();

private:
    RH_RF95 rfm;
    uint8_t rstPin;
};

#endif // RADIO_LINK_H