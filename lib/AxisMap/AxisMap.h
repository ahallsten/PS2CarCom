#ifndef AXISMAP_H
#define AXISMAP_H

#include <Arduino.h>  // if you need map(), constrain(), etc.

struct AxisMap {
  uint8_t low_dead;
  uint8_t low_from;
  uint8_t low_to;
  uint8_t high_dead;
  uint8_t high_from;
  uint8_t high_to;
  uint8_t out_min;
  uint8_t out_max;
};

uint8_t mapAxis(uint8_t raw, const AxisMap &m);
int16_t mapAxisSigned(uint8_t raw, const AxisMap &m);

#endif
