#pragma once

#include <stdint.h>

class RadarFilter {
 public:
  static bool shouldFocus(uint16_t previousCm,
                          uint16_t measuredCm,
                          bool previousFresh,
                          uint16_t focusMaximumCm,
                          uint16_t minimumDropCm);
};
