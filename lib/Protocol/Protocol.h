#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include "ControllerState.h"

enum PacketType : uint8_t {
  PACKET_CONTROL = 1,
  PACKET_STATUS  = 2,
};

struct ControlPacket {
  uint8_t type = PACKET_CONTROL;
  ControllerState state;
};

struct StatusPacket {
  uint8_t type = PACKET_STATUS;
  uint8_t linkOk = 0;
  int16_t rssiRaw = 0;
  int16_t rssiSmooth = 0;
  float batteryV = 0.0f;
  uint8_t motorPct[4] = {0, 0, 0, 0};
};

inline int16_t smoothRssi(int16_t sample, int16_t prev) {
  // Simple EMA: alpha = 0.25 (new contributes 1/4)
  return prev + ((sample - prev) / 4);
}

#endif
