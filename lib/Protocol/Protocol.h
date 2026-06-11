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
static const uint8_t PROTOCOL_VERSION = 3;

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

  /** Receiver parking brake engaged. Meaningful in the receiver->transmitter direction. */
  STATUS_FLAG_PARKING_BRAKE = 0x04,

  /** Receiver drive output enabled. Meaningful in the receiver->transmitter direction. */
  STATUS_FLAG_DRIVE_ENABLED = 0x08,

  /** Receiver tank-drive mode active. Meaningful in the receiver->transmitter direction. */
  STATUS_FLAG_TANK_MODE = 0x10,
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
 * @brief Decoded status/telemetry packet payload (primarily receiver->transmitter).
 *
 * Carries link health, control-mode flags, RSSI, battery telemetry, per-motor
 * PWM commands, and BTS7960 current-sense ADC readings so the transmitter can
 * render a dashboard. The transmitter also sends a status packet back; in that
 * direction only linkOk/controllerPresent/rssi/batteryMv are meaningful and the
 * receiver-specific fields are left zero.
 */
struct StatusMessage {
  /** true when the reporting side considers its control link healthy. */
  bool linkOk = false;

  /** true when a transmitter-side controller is present. */
  bool controllerPresent = false;

  /** Receiver parking brake engaged (receiver->transmitter direction). */
  bool parkingBrake = false;

  /** Receiver drive output enabled (receiver->transmitter direction). */
  bool driveEnabled = false;

  /** Receiver tank-drive mode active (receiver->transmitter direction). */
  bool tankMode = false;

  /**
   * @brief Last control sequence number the sender received (lightweight ACK).
   *
   * The receiver sets this to the seq of the most recent control packet it
   * decoded, so the transmitter can confirm which control packets made it
   * across without a per-packet handshake. Unused/0 in the TX->RX direction.
   */
  uint8_t ackSeq = 0;

  /** Last raw RSSI sample in dBm-style RadioHead units. */
  int16_t rssiRaw = 0;

  /** Smoothed RSSI value from smoothRssi(). */
  int16_t rssiSmooth = 0;

  /** Battery estimate in millivolts. Scaling depends on the board-side sense circuit. */
  uint16_t batteryMv = 0;

  /** Raw battery ADC reading (0..1023) before scaling, for calibration/diagnostics. */
  uint16_t batteryRaw = 0;

  /** Signed motor PWM commands being applied (-255..255): FL, FR, RL, RR. */
  int16_t motorPwm[4] = {0, 0, 0, 0};

  /**
   * @brief BTS7960 current-sense raw ADC readings (0..1023).
   *
   * Order: FL L_IS, FL R_IS, FR L_IS, FR R_IS, RL L_IS, RL R_IS, RR L_IS, RR R_IS.
   * Pins routed through the MCP23017 cannot be analog-read and report 0.
   */
  uint16_t currentSense[8] = {0, 0, 0, 0, 0, 0, 0, 0};
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

/**
 * @brief Encoded byte length of a status packet.
 *
 * Wire layout (little-endian):
 *   [0]      protocol version
 *   [1]      packet type (PACKET_STATUS)
 *   [2]      flags (StatusFlags)
 *   [3]      ackSeq
 *   [4..5]   rssiRaw (i16)
 *   [6..7]   rssiSmooth (i16)
 *   [8..9]   batteryMv (u16)
 *   [10..11] batteryRaw (u16)
 *   [12..19] motorPwm[4] (i16 each): FL, FR, RL, RR
 *   [20..35] currentSense[8] (u16 each): FL_L, FL_R, FR_L, FR_R, RL_L, RL_R, RR_L, RR_R
 */
static const uint8_t STATUS_MESSAGE_SIZE = 36;

/** @brief Largest packet buffer needed by the current protocol. */
static const uint8_t MAX_WIRE_PACKET_SIZE = STATUS_MESSAGE_SIZE;

// Keep the documented wire layout and the size constant in agreement. If you
// change StatusMessage fields, update both the offsets above and this value.
static_assert(STATUS_MESSAGE_SIZE == 36, "STATUS_MESSAGE_SIZE must match the documented wire layout");
static_assert(MAX_WIRE_PACKET_SIZE >= CONTROL_MESSAGE_SIZE, "wire buffer must hold a control packet");

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
  if (msg.parkingBrake) out[2] |= STATUS_FLAG_PARKING_BRAKE;
  if (msg.driveEnabled) out[2] |= STATUS_FLAG_DRIVE_ENABLED;
  if (msg.tankMode) out[2] |= STATUS_FLAG_TANK_MODE;
  out[3] = msg.ackSeq;
  writeI16LE(out + 4, msg.rssiRaw);
  writeI16LE(out + 6, msg.rssiSmooth);
  writeU16LE(out + 8, msg.batteryMv);
  writeU16LE(out + 10, msg.batteryRaw);
  for (uint8_t i = 0; i < 4; ++i) {
    writeI16LE(out + 12 + (i * 2), msg.motorPwm[i]);
  }
  for (uint8_t i = 0; i < 8; ++i) {
    writeU16LE(out + 20 + (i * 2), msg.currentSense[i]);
  }
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
  msg.parkingBrake = (data[2] & STATUS_FLAG_PARKING_BRAKE) != 0;
  msg.driveEnabled = (data[2] & STATUS_FLAG_DRIVE_ENABLED) != 0;
  msg.tankMode = (data[2] & STATUS_FLAG_TANK_MODE) != 0;
  msg.ackSeq = data[3];
  msg.rssiRaw = readI16LE(data + 4);
  msg.rssiSmooth = readI16LE(data + 6);
  msg.batteryMv = readU16LE(data + 8);
  msg.batteryRaw = readU16LE(data + 10);
  for (uint8_t i = 0; i < 4; ++i) {
    msg.motorPwm[i] = readI16LE(data + 12 + (i * 2));
  }
  for (uint8_t i = 0; i < 8; ++i) {
    msg.currentSense[i] = readU16LE(data + 20 + (i * 2));
  }
  return true;
}

#endif
