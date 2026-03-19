#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include <string.h>
#include "ControllerState.h"

static const uint8_t PROTOCOL_VERSION = 1;

enum PacketType : uint8_t {
  PACKET_CONTROL = 1,
  PACKET_STATUS  = 2,
};

enum ControlFlags : uint8_t {
  CONTROL_FLAG_CONTROLLER_PRESENT = 0x01,
};

enum StatusFlags : uint8_t {
  STATUS_FLAG_LINK_OK = 0x01,
  STATUS_FLAG_CONTROLLER_PRESENT = 0x02,
};

struct ControlMessage {
  uint8_t seq = 0;
  bool controllerPresent = false;
  ControllerState state = { 0, 127, 127, 127, 127 };
};

struct StatusMessage {
  bool linkOk = false;
  bool controllerPresent = false;
  int16_t rssiRaw = 0;
  int16_t rssiSmooth = 0;
  uint16_t batteryMv = 0;
  uint8_t motorPct[4] = {0, 0, 0, 0};
};

inline int16_t smoothRssi(int16_t sample, int16_t prev) {
  // Simple EMA: alpha = 0.25 (new contributes 1/4)
  return prev + ((sample - prev) / 4);
}

static const ControllerState kNeutralControllerState = { 0, 127, 127, 127, 127 };
static const uint8_t CONTROL_MESSAGE_SIZE = 10;
static const uint8_t STATUS_MESSAGE_SIZE = 13;
static const uint8_t MAX_WIRE_PACKET_SIZE = STATUS_MESSAGE_SIZE;

inline void writeU16LE(uint8_t *dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline uint16_t readU16LE(const uint8_t *src) {
  return static_cast<uint16_t>(src[0]) |
         (static_cast<uint16_t>(src[1]) << 8);
}

inline void writeI16LE(uint8_t *dst, int16_t value) {
  writeU16LE(dst, static_cast<uint16_t>(value));
}

inline int16_t readI16LE(const uint8_t *src) {
  return static_cast<int16_t>(readU16LE(src));
}

inline void encodeControlMessage(const ControlMessage &msg,
                                 uint8_t out[CONTROL_MESSAGE_SIZE]) {
  out[0] = PROTOCOL_VERSION;
  out[1] = PACKET_CONTROL;
  out[2] = msg.seq;
  out[3] = msg.controllerPresent ? CONTROL_FLAG_CONTROLLER_PRESENT : 0;
  writeU16LE(out + 4, msg.state.buttonWord);
  out[6] = msg.state.leftX;
  out[7] = msg.state.leftY;
  out[8] = msg.state.rightX;
  out[9] = msg.state.rightY;
}

inline bool decodeControlMessage(const uint8_t *data, uint8_t len, ControlMessage &msg) {
  if (len != CONTROL_MESSAGE_SIZE) return false;
  if (data[0] != PROTOCOL_VERSION || data[1] != PACKET_CONTROL) return false;

  msg.seq = data[2];
  msg.controllerPresent = (data[3] & CONTROL_FLAG_CONTROLLER_PRESENT) != 0;
  msg.state.buttonWord = readU16LE(data + 4);
  msg.state.leftX = data[6];
  msg.state.leftY = data[7];
  msg.state.rightX = data[8];
  msg.state.rightY = data[9];
  return true;
}

inline void encodeStatusMessage(const StatusMessage &msg,
                                uint8_t out[STATUS_MESSAGE_SIZE]) {
  out[0] = PROTOCOL_VERSION;
  out[1] = PACKET_STATUS;
  out[2] = 0;
  if (msg.linkOk) out[2] |= STATUS_FLAG_LINK_OK;
  if (msg.controllerPresent) out[2] |= STATUS_FLAG_CONTROLLER_PRESENT;
  writeI16LE(out + 3, msg.rssiRaw);
  writeI16LE(out + 5, msg.rssiSmooth);
  writeU16LE(out + 7, msg.batteryMv);
  memcpy(out + 9, msg.motorPct, sizeof(msg.motorPct));
}

inline bool decodeStatusMessage(const uint8_t *data, uint8_t len, StatusMessage &msg) {
  if (len != STATUS_MESSAGE_SIZE) return false;
  if (data[0] != PROTOCOL_VERSION || data[1] != PACKET_STATUS) return false;

  msg.linkOk = (data[2] & STATUS_FLAG_LINK_OK) != 0;
  msg.controllerPresent = (data[2] & STATUS_FLAG_CONTROLLER_PRESENT) != 0;
  msg.rssiRaw = readI16LE(data + 3);
  msg.rssiSmooth = readI16LE(data + 5);
  msg.batteryMv = readU16LE(data + 7);
  memcpy(msg.motorPct, data + 9, sizeof(msg.motorPct));
  return true;
}

#endif
