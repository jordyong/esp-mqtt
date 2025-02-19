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
