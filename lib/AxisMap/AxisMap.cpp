#include "AxisMap.h"

uint16_t mapAxis(uint8_t raw, const AxisMap &m) {
  if (raw < m.low_dead) {
    long mapped = map(raw, m.low_from, m.low_to, m.out_min, m.out_max);
    return static_cast<uint16_t>(constrain(mapped, static_cast<long>(m.out_min), static_cast<long>(m.out_max)));
  } else if (raw > m.high_dead) {
    long mapped = map(raw, m.high_from, m.high_to, m.out_min, m.out_max);
    return static_cast<uint16_t>(constrain(mapped, static_cast<long>(m.out_min), static_cast<long>(m.out_max)));
  } else {
    return 0;
  }
}

int16_t mapAxisSigned(uint8_t raw, const AxisMap &m) {
  if (raw < m.low_dead) {
    long mapped = map(raw, m.low_from, m.low_to, 0, m.out_max);
    return static_cast<int16_t>(constrain(mapped, 0L, static_cast<long>(m.out_max)));
  } else if (raw > m.high_dead) {
    long mapped = map(raw, m.high_from, m.high_to, 0, m.out_max);
    return -static_cast<int16_t>(constrain(mapped, 0L, static_cast<long>(m.out_max)));
  } else {
    return 0;
  }
}
