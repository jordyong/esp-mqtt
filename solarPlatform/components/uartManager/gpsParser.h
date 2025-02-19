#pragma once

#include "models.h"

#ifdef __cplusplus
extern "C" {
#endif

void ddmmIntoDD(float *target);
void nmeaTermHandler(gpsInfo *gi, parserInfo *pi);
int uartParser(const char c, const int currCnt, parserInfo *pi);

#ifdef __cplusplus
}
#endif
