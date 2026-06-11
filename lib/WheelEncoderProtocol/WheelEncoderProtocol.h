#ifndef WHEEL_ENCODER_PROTOCOL_H
#define WHEEL_ENCODER_PROTOCOL_H

#include <Arduino.h>

static const uint8_t WHEEL_ENCODER_PROTOCOL_VERSION = 1;
static const uint8_t WHEEL_ENCODER_SLAVE_I2C_ADDRESS = 0x12;
static const uint8_t WHEEL_ENCODER_PACKET_SIZE = 28;

enum WheelEncoderStatusFlags : uint8_t {
  WHEEL_ENCODER_STATUS_SENSOR_FAULT = 0x01,
  WHEEL_ENCODER_STATUS_TCA_FAULT = 0x02,
};

struct WheelEncoderMessage {
  uint8_t sequence = 0;
  uint8_t status = 0;
  uint16_t raw[4] = {0, 0, 0, 0};
  int16_t delta[4] = {0, 0, 0, 0};
  int16_t speedCountsPerSecond[4] = {0, 0, 0, 0};
};

inline uint8_t wheelEncoderChecksum(const uint8_t *data, uint8_t lenWithoutChecksum) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < lenWithoutChecksum; ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  return static_cast<uint8_t>(0U - sum);
}

inline void encodeWheelEncoderMessage(const WheelEncoderMessage &msg,
                                      uint8_t out[WHEEL_ENCODER_PACKET_SIZE]) {
  out[0] = WHEEL_ENCODER_PROTOCOL_VERSION;
  out[1] = msg.sequence;
  out[2] = msg.status;

  uint8_t offset = 3;
  for (uint8_t i = 0; i < 4; ++i) {
    out[offset++] = static_cast<uint8_t>(msg.raw[i] & 0xFF);
    out[offset++] = static_cast<uint8_t>(msg.raw[i] >> 8);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    uint16_t value = static_cast<uint16_t>(msg.delta[i]);
    out[offset++] = static_cast<uint8_t>(value & 0xFF);
    out[offset++] = static_cast<uint8_t>(value >> 8);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    uint16_t value = static_cast<uint16_t>(msg.speedCountsPerSecond[i]);
    out[offset++] = static_cast<uint8_t>(value & 0xFF);
    out[offset++] = static_cast<uint8_t>(value >> 8);
  }

  out[WHEEL_ENCODER_PACKET_SIZE - 1] =
    wheelEncoderChecksum(out, WHEEL_ENCODER_PACKET_SIZE - 1);
}

inline bool decodeWheelEncoderMessage(const uint8_t *data,
                                      uint8_t len,
                                      WheelEncoderMessage &msg) {
  if (len != WHEEL_ENCODER_PACKET_SIZE) return false;
  if (data[0] != WHEEL_ENCODER_PROTOCOL_VERSION) return false;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < WHEEL_ENCODER_PACKET_SIZE; ++i) {
    sum = static_cast<uint8_t>(sum + data[i]);
  }
  if (sum != 0) return false;

  msg.sequence = data[1];
  msg.status = data[2];
  uint8_t offset = 3;
  for (uint8_t i = 0; i < 4; ++i) {
    msg.raw[i] = static_cast<uint16_t>(data[offset]) |
                 (static_cast<uint16_t>(data[offset + 1]) << 8);
    offset += 2;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    uint16_t value = static_cast<uint16_t>(data[offset]) |
                     (static_cast<uint16_t>(data[offset + 1]) << 8);
    msg.delta[i] = static_cast<int16_t>(value);
    offset += 2;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    uint16_t value = static_cast<uint16_t>(data[offset]) |
                     (static_cast<uint16_t>(data[offset + 1]) << 8);
    msg.speedCountsPerSecond[i] = static_cast<int16_t>(value);
    offset += 2;
  }

  return true;
}

static_assert(WHEEL_ENCODER_PACKET_SIZE == 28, "wheel encoder packet layout changed");

#endif
