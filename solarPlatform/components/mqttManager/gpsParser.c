#include <math.h>

#include "gpsParser.h"

float ddmmIntoDD(const float target) {
  float ddmm = target;

  int deg = ((int)ddmm) / 100;
  float mins = (ddmm - ((int)ddmm) / 100 * 100);

  return deg + (mins / 60);
}

float DDToddmm(const float target) {
  float dd = target;
  int deg = (int)floor(fabs(dd));
  float mins = (fabs(dd) - deg) * 60.0;

  return (deg * 100) + mins;
}
