#include "RadioLink.h"

// Construct the radio link with provided pin assignments.
RadioLink::RadioLink(uint8_t csPin, uint8_t intPin, uint8_t rstPin)
: rfm(csPin, intPin),
  rstPin(rstPin)
{}

// Initialises the radio and sets the operating frequency.
bool RadioLink::init(float freq) {
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, HIGH);
    if (!rfm.init()) {
        return false;
    }
    rfm.setFrequency(freq);
    return true;
}

bool RadioLink::sendPacket(const void* data, uint8_t len) {
    // Cast the data to a byte pointer and send it.
    bool ok = rfm.send((const uint8_t*)data, len);
    // Block until the packet has actually been sent.
    rfm.waitPacketSent();
    return ok;
}

bool RadioLink::recvPacket(void* buffer, uint8_t* len) {
    // Attempt to receive a packet into the provided buffer.
    return rfm.recv((uint8_t*)buffer, len);
}

bool RadioLink::available() {
    return rfm.available();
}