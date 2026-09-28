#include "RadarFilter.h"

bool RadarFilter::shouldFocus(const uint16_t previousCm,
                              const uint16_t measuredCm,
                              const bool previousFresh,
                              const uint16_t focusMaximumCm,
                              const uint16_t minimumDropCm) {
  if (measuredCm > focusMaximumCm) {
    return false;
  }
  if (!previousFresh) {
    return true;
  }
  return measuredCm < previousCm &&
         static_cast<uint16_t>(previousCm - measuredCm) >=
             minimumDropCm;
}
