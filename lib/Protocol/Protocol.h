#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include "ControllerState.h"
#include "VehicleLayout.h"

static const uint8_t PROTOCOL_VERSION = 5;

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
  STATUS_FLAG_PARKING_BRAKE = 0x04,
  STATUS_FLAG_DRIVE_ENABLED = 0x08,
  STATUS_FLAG_TANK_MODE = 0x10,
};

struct ControlMessage {
  uint8_t seq = 0;
  bool controllerPresent = false;
  ControllerState state = { 0, 127, 127, 127, 127 };
};

struct StatusMessage {
  bool linkOk = false;
  bool controllerPresent = false;
  bool parkingBrake = false;
  bool driveEnabled = false;
  bool tankMode = false;
  uint8_t ackSeq = 0;
  int16_t rssiRaw = 0;
  int16_t rssiSmooth = 0;

  /** Receiver pack total voltage in millivolts. In TX->RX status, this is TX battery mV. */
  uint16_t batteryTotalMv = 0;

  /** Receiver midpoint voltage in millivolts. Zero when not available. */
  uint16_t batteryMidpointMv = 0;

  /** Signed PWM commands being applied (-255..255): FL, FR, RL, RR, STEER. */
  int16_t motorPwm[VEHICLE_MOTOR_COUNT] = {0, 0, 0, 0, 0};

  /** Raw native ADC current-sense readings: FL_L, FL_R, FR_L, FR_R, RL_L, RL_R, RR_L, RR_R, STEER_L, STEER_R. */
  uint16_t currentSense[VEHICLE_CURRENT_SENSE_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  /** AS5600 raw angle, 0..4095. */
  uint16_t steeringRawAngle = 0;

  /** Steering actuator position in unwrapped AS5600 counts relative to home. */
  int32_t steeringPositionCounts = 0;

  /** Steering actuator target in unwrapped AS5600 counts relative to home. */
  int32_t steeringTargetCounts = 0;

  /** Steering actuator state enum value. */
  uint8_t steeringState = 0;

  /** Steering flags: calibration valid, homed, home switch active, current limit active. */
  uint8_t steeringFlags = 0;

  /** Steering fault bitfield. */
  uint16_t steeringFaultFlags = 0;

  /** Max raw ADC count from the steering BTS7960 current-sense pins. */
  uint16_t steeringCurrentRaw = 0;
};

inline int16_t smoothRssi(int16_t sample, int16_t prev) {
  return prev + ((sample - prev) / 4);
}

static const ControllerState kNeutralControllerState = { 0, 127, 127, 127, 127 };

static const uint8_t CONTROL_MESSAGE_SIZE = 10;

/**
 * Status packet wire layout, little-endian:
 *   [0]      protocol version
 *   [1]      packet type (PACKET_STATUS)
 *   [2]      flags (StatusFlags)
 *   [3]      ackSeq
 *   [4..5]   rssiRaw (i16)
 *   [6..7]   rssiSmooth (i16)
 *   [8..9]   batteryTotalMv (u16)
 *   [10..11] batteryMidpointMv (u16)
 *   [12..21] motorPwm[5] (i16 each): FL, FR, RL, RR, STEER
 *   [22..41] currentSense[10] (u16 each): FL_L, FL_R, FR_L, FR_R, RL_L, RL_R, RR_L, RR_R, STEER_L, STEER_R
 *   [42..43] steeringRawAngle (u16, AS5600 raw 0..4095)
 *   [44..47] steeringPositionCounts (i32, AS5600 counts relative to home)
 *   [48..51] steeringTargetCounts (i32, AS5600 counts relative to home)
 *   [52]      steeringState (u8)
 *   [53]      steeringFlags (u8)
 *   [54..55] steeringFaultFlags (u16)
 *   [56..57] steeringCurrentRaw (u16)
 */
static const uint8_t STATUS_MESSAGE_SIZE = 58;
static const uint8_t MAX_WIRE_PACKET_SIZE = STATUS_MESSAGE_SIZE;

static_assert(STATUS_MESSAGE_SIZE == 58, "STATUS_MESSAGE_SIZE must match the documented wire layout");
static_assert(MAX_WIRE_PACKET_SIZE >= CONTROL_MESSAGE_SIZE, "wire buffer must hold a control packet");

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

inline void writeU32LE(uint8_t *dst, uint32_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFF);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  dst[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  dst[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

inline uint32_t readU32LE(const uint8_t *src) {
  return static_cast<uint32_t>(src[0]) |
         (static_cast<uint32_t>(src[1]) << 8) |
         (static_cast<uint32_t>(src[2]) << 16) |
         (static_cast<uint32_t>(src[3]) << 24);
}

inline void writeI32LE(uint8_t *dst, int32_t value) {
  writeU32LE(dst, static_cast<uint32_t>(value));
}

inline int32_t readI32LE(const uint8_t *src) {
  return static_cast<int32_t>(readU32LE(src));
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
  if (msg.parkingBrake) out[2] |= STATUS_FLAG_PARKING_BRAKE;
  if (msg.driveEnabled) out[2] |= STATUS_FLAG_DRIVE_ENABLED;
  if (msg.tankMode) out[2] |= STATUS_FLAG_TANK_MODE;
  out[3] = msg.ackSeq;
  writeI16LE(out + 4, msg.rssiRaw);
  writeI16LE(out + 6, msg.rssiSmooth);
  writeU16LE(out + 8, msg.batteryTotalMv);
  writeU16LE(out + 10, msg.batteryMidpointMv);
  for (uint8_t i = 0; i < VEHICLE_MOTOR_COUNT; ++i) {
    writeI16LE(out + 12 + (i * 2), msg.motorPwm[i]);
  }
  for (uint8_t i = 0; i < VEHICLE_CURRENT_SENSE_COUNT; ++i) {
    writeU16LE(out + 22 + (i * 2), msg.currentSense[i]);
  }
  writeU16LE(out + 42, msg.steeringRawAngle);
  writeI32LE(out + 44, msg.steeringPositionCounts);
  writeI32LE(out + 48, msg.steeringTargetCounts);
  out[52] = msg.steeringState;
  out[53] = msg.steeringFlags;
  writeU16LE(out + 54, msg.steeringFaultFlags);
  writeU16LE(out + 56, msg.steeringCurrentRaw);
}

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
  msg.batteryTotalMv = readU16LE(data + 8);
  msg.batteryMidpointMv = readU16LE(data + 10);
  for (uint8_t i = 0; i < VEHICLE_MOTOR_COUNT; ++i) {
    msg.motorPwm[i] = readI16LE(data + 12 + (i * 2));
  }
  for (uint8_t i = 0; i < VEHICLE_CURRENT_SENSE_COUNT; ++i) {
    msg.currentSense[i] = readU16LE(data + 22 + (i * 2));
  }
  msg.steeringRawAngle = readU16LE(data + 42);
  msg.steeringPositionCounts = readI32LE(data + 44);
  msg.steeringTargetCounts = readI32LE(data + 48);
  msg.steeringState = data[52];
  msg.steeringFlags = data[53];
  msg.steeringFaultFlags = readU16LE(data + 54);
  msg.steeringCurrentRaw = readU16LE(data + 56);
  return true;
}

#endif
