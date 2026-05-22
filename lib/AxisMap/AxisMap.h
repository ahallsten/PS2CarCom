#ifndef AXISMAP_H
#define AXISMAP_H

#include <Arduino.h>

/**
 * @brief Defines how one analog stick axis is converted into a PWM command.
 *
 * The receiver uses this to apply a center dead zone and map the low-side and
 * high-side stick travel into an output range. The low and high halves are
 * configured separately because real controller sticks often do not center or
 * travel symmetrically.
 */
struct AxisMap {
  /** Raw values below this threshold are treated as low-side movement. */
  uint8_t low_dead;

  /** Raw input value corresponding to the start of the low-side map range. */
  uint8_t low_from;

  /** Raw input value corresponding to the end of the low-side map range. */
  uint8_t low_to;

  /** Raw values above this threshold are treated as high-side movement. */
  uint8_t high_dead;

  /** Raw input value corresponding to the start of the high-side map range. */
  uint8_t high_from;

  /** Raw input value corresponding to the end of the high-side map range. */
  uint8_t high_to;

  /** Minimum output value returned by mapAxis(). */
  uint8_t out_min;

  /** Maximum output value returned by mapAxis() and mapAxisSigned(). */
  uint8_t out_max;
};

/**
 * @brief Map a raw 8-bit axis value into an unsigned output range.
 *
 * Values inside the configured dead zone return 0. Values outside the dead
 * zone are mapped through the matching low-side or high-side calibration range.
 *
 * @param raw Raw controller axis value, normally 0..255.
 * @param m Axis calibration and output range.
 * @return Unsigned mapped value, or 0 while in the dead zone.
 */
uint8_t mapAxis(uint8_t raw, const AxisMap &m);

/**
 * @brief Map a raw 8-bit axis value into a signed PWM command.
 *
 * Low-side movement returns a positive value, high-side movement returns a
 * negative value, and dead-zone values return 0.
 *
 * @param raw Raw controller axis value, normally 0..255.
 * @param m Axis calibration and output range.
 * @return Signed output in the range -m.out_max..m.out_max.
 */
int16_t mapAxisSigned(uint8_t raw, const AxisMap &m);

#endif
