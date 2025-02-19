#include <stdlib.h>

#include "gpsParser.h"

void ddmmIntoDD(float *target) {
  float ddmm = *target;

  int dd = ((int)ddmm) / 100;
  float mm = (ddmm - ((int)ddmm) / 100 * 100);
  *target = dd + (mm / 60);
}

void nmeaTermHandler(gpsInfo *gi, parserInfo *pi) {
  switch (pi->termIndex) {
  case 2: {
    gi->lat = atof(pi->term);
    ddmmIntoDD(&gi->lat);
    break;
  }
  case 4: {
    gi->lng = atof(pi->term);
    ddmmIntoDD(&gi->lng);
    break;
  }
  default: {
    break;
  }
  }
}

int uartParser(const char c, const int currCnt, parserInfo *pi) {
  switch (c) {
  case ',': {
    pi->termIndex++;
    pi->term[pi->termOffset++] = '\0';
    pi->termOffset = 0;
    return 1;
    break;
  }
  case '$': {
    pi->currTerm = pi->termOffset = 0;
    pi->termOffset = currCnt;
    break;
  }
  default: {
    if (pi->termOffset < currCnt) {
      pi->term[pi->termOffset++] = c;
    }
    break;
  }
  }

  return 0;
}
