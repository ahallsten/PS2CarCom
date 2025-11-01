#ifndef AXISMAP_H
#define AXISMAP_H

#include <Arduino.h>  // if you need map(), constrain(), etc.

/**
 * @struct
 * @brief Represents the values that define the dead zone.
 *
 * This structure stores the values needed to cleanly map
 * the positive and negative range of an analog stick axis
 * to values that control a single range output pwm
 *
 */

struct AxisMap {
  uint8_t low_dead;  /**< @brief The y-coordinate of the point. */
  uint8_t low_from;  /**< @brief The y-coordinate of the point. */
  uint8_t low_to;    /**< @brief The y-coordinate of the point. */
  uint8_t high_dead; /**< @brief The y-coordinate of the point. */
  uint8_t high_from; /**< @brief The y-coordinate of the point. */
  uint8_t high_to;   /**< @brief The y-coordinate of the point. */
  uint8_t out_min;   /**< @brief The y-coordinate of the point. */
  uint8_t out_max;   /**< @brief The y-coordinate of the point. */
};

uint8_t mapAxis(uint8_t raw, const AxisMap &m);
int16_t mapAxisSigned(uint8_t raw, const AxisMap &m);

#endif
