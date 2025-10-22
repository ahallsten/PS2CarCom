#include "AxisMap.h"

uint8_t mapAxis(uint8_t raw, const AxisMap &m) {
  if (raw < m.low_dead)
    return constrain(map(raw, m.low_from, m.low_to, m.out_min, m.out_max), m.out_min, m.out_max);
  else if (raw > m.high_dead)
    return constrain(map(raw, m.high_from, m.high_to, m.out_min, m.out_max), m.out_min, m.out_max);
  else
    return 0;
}

int16_t mapAxisSigned(uint8_t raw, const AxisMap &m) {
  if (raw < m.low_dead)
    return constrain(map(raw, m.low_from, m.low_to, 0, m.out_max), 0, m.out_max);
  else if (raw > m.high_dead)
    return -constrain(map(raw, m.high_from, m.high_to, 0, m.out_max), 0, m.out_max);
  else
    return 0;
}
