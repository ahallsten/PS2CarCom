#ifndef VEHICLE_LAYOUT_H
#define VEHICLE_LAYOUT_H

#include <Arduino.h>

/** Number of BTS7960-driven outputs reported in telemetry. */
static const uint8_t VEHICLE_MOTOR_COUNT = 5;

/** Each BTS7960 exposes L_IS and R_IS current-sense pins. */
static const uint8_t VEHICLE_CURRENT_SENSE_COUNT = VEHICLE_MOTOR_COUNT * 2;

enum VehicleMotorIndex : uint8_t {
  MOTOR_INDEX_FL = 0,
  MOTOR_INDEX_FR = 1,
  MOTOR_INDEX_RL = 2,
  MOTOR_INDEX_RR = 3,
  MOTOR_INDEX_STEER = 4,
};

enum VehicleCurrentSenseIndex : uint8_t {
  CURRENT_INDEX_FL_L = 0,
  CURRENT_INDEX_FL_R = 1,
  CURRENT_INDEX_FR_L = 2,
  CURRENT_INDEX_FR_R = 3,
  CURRENT_INDEX_RL_L = 4,
  CURRENT_INDEX_RL_R = 5,
  CURRENT_INDEX_RR_L = 6,
  CURRENT_INDEX_RR_R = 7,
  CURRENT_INDEX_STEER_L = 8,
  CURRENT_INDEX_STEER_R = 9,
};

#endif
