#pragma once

typedef struct parserInfo_tag {
  int termOffset;
  int termIndex;
  int currTerm;
  char term[20];
} parserInfo;

typedef struct gpsInfo_tag {
  float lat;
  float lng;
} gpsInfo;

#ifdef __cplusplus
extern "C" {
#endif

void ddmmIntoDD(float *target);
void nmeaTermHandler(gpsInfo *gi, parserInfo *pi);
int uartParser(const char c, const int currCnt, parserInfo *pi);

#ifdef __cplusplus
}
#endif
