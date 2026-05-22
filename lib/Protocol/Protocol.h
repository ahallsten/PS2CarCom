#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include <string.h>
#include "ControllerState.h"

/**
 * @brief Wire protocol version byte used at the start of every packet.
 *
 * Change this only when both transmitter and receiver decode paths are updated
 * or when backward-compatible decode logic is added.
 */
static const uint8_t PROTOCOL_VERSION = 1;

/**
 * @brief Packet type byte values used after the protocol version byte.
 */
enum PacketType : uint8_t {
  /** Control packet sent primarily from transmitter to receiver. */
  PACKET_CONTROL = 1,

  /** Status packet sent by both sides for link/telemetry reporting. */
  PACKET_STATUS  = 2,
};

/**
 * @brief Bit flags stored in ControlMessage packets.
 */
enum ControlFlags : uint8_t {
  /** Set when the transmitter currently has a valid PS2 controller. */
  CONTROL_FLAG_CONTROLLER_PRESENT = 0x01,
};

/**
 * @brief Bit flags stored in StatusMessage packets.
 */
enum StatusFlags : uint8_t {
  /** Set when the reporting side considers the opposite link healthy. */
  STATUS_FLAG_LINK_OK = 0x01,

  /** Set when the transmitter-side controller is present. */
  STATUS_FLAG_CONTROLLER_PRESENT = 0x02,
};

/**
 * @brief Decoded control packet payload.
 *
 * Control packets carry the transmitter sequence number, whether a controller
 * is present, and the latest PS2 controller state.
 */
struct ControlMessage {
  /** Monotonic 8-bit sequence number from the transmitter. */
  uint8_t seq = 0;

  /** true when the transmitter has a connected/readable controller. */
  bool controllerPresent = false;

  /** Controller state to apply; neutral is used by default. */
  ControllerState state = { 0, 127, 127, 127, 127 };
};

/**
 * @brief Decoded status packet payload.
 *
 * Status packets report link health, RSSI, battery telemetry, and receiver
 * motor-command percentages.
 */
struct StatusMessage {
  /** true when the reporting side considers its control link healthy. */
  bool linkOk = false;

  /** true when a transmitter-side controller is present. */
  bool controllerPresent = false;

  /** Last raw RSSI sample in dBm-style RadioHead units. */
  int16_t rssiRaw = 0;

  /** Smoothed RSSI value from smoothRssi(). */
  int16_t rssiSmooth = 0;

  /** Battery estimate in millivolts. Scaling depends on the board-side sense circuit. */
  uint16_t batteryMv = 0;

  /** Motor command magnitudes as percentages: FL, FR, RL, RR. */
  uint8_t motorPct[4] = {0, 0, 0, 0};
};

/**
 * @brief Smooth an RSSI sample with a small integer exponential moving average.
 *
 * @param sample New RSSI sample.
 * @param prev Previous smoothed value.
 * @return Updated smoothed RSSI value with alpha approximately 0.25.
 */
inline int16_t smoothRssi(int16_t sample, int16_t prev) {
  // Simple EMA: alpha = 0.25 (new contributes 1/4)
  return prev + ((sample - prev) / 4);
}

/** @brief Neutral controller state used for failsafe and no-controller cases. */
static const ControllerState kNeutralControllerState = { 0, 127, 127, 127, 127 };

/** @brief Encoded byte length of a control packet. */
static const uint8_t CONTROL_MESSAGE_SIZE = 10;

/** @brief Encoded byte length of a status packet. */
static const uint8_t STATUS_MESSAGE_SIZE = 13;

/** @brief Largest packet buffer needed by the current protocol. */
static const uint8_t MAX_WIRE_PACKET_SIZE = STATUS_MESSAGE_SIZE;

/**
 * @brief Write a 16-bit unsigned integer in little-endian byte order.
 *
 * @param dst Destination pointer to at least two bytes.
 * @param value Value to encode.
 */
inline void writeU16LE(uint8_t *dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

/**
 * @brief Read a 16-bit unsigned integer from little-endian bytes.
 *
 * @param src Source pointer to at least two bytes.
 * @return Decoded unsigned 16-bit value.
 */
inline uint16_t readU16LE(const uint8_t *src) {
  return static_cast<uint16_t>(src[0]) |
         (static_cast<uint16_t>(src[1]) << 8);
}

/**
 * @brief Write a 16-bit signed integer in little-endian byte order.
 *
 * @param dst Destination pointer to at least two bytes.
 * @param value Value to encode.
 */
inline void writeI16LE(uint8_t *dst, int16_t value) {
  writeU16LE(dst, static_cast<uint16_t>(value));
}

/**
 * @brief Read a 16-bit signed integer from little-endian bytes.
 *
 * @param src Source pointer to at least two bytes.
 * @return Decoded signed 16-bit value.
 */
inline int16_t readI16LE(const uint8_t *src) {
  return static_cast<int16_t>(readU16LE(src));
}

/**
 * @brief Encode a ControlMessage into its fixed-width wire packet.
 *
 * @param msg Decoded control message fields.
 * @param out Destination array of CONTROL_MESSAGE_SIZE bytes.
 */
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

/**
 * @brief Decode a fixed-width wire packet into a ControlMessage.
 *
 * Packet length, protocol version, and packet type must all match. Invalid
 * packets leave msg partially unspecified and return false.
 *
 * @param data Source packet bytes.
 * @param len Number of source bytes.
 * @param msg Destination decoded message.
 * @return true when the packet is a valid control message.
 */
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

/**
 * @brief Encode a StatusMessage into its fixed-width wire packet.
 *
 * @param msg Decoded status message fields.
 * @param out Destination array of STATUS_MESSAGE_SIZE bytes.
 */
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

/**
 * @brief Decode a fixed-width wire packet into a StatusMessage.
 *
 * Packet length, protocol version, and packet type must all match. Invalid
 * packets leave msg partially unspecified and return false.
 *
 * @param data Source packet bytes.
 * @param len Number of source bytes.
 * @param msg Destination decoded message.
 * @return true when the packet is a valid status message.
 */
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
