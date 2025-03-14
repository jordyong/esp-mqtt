#ifndef JSONPARSER_H
#define JSONPARSER_H

#include "freertos/idf_additions.h"
#include "models.h"

#ifdef __cplusplus
extern "C" {
#endif

void json_uartParser(const char *c);
void json_mqttParser(const char *c);

void json_GPStoMQTT(float latitude, float longitude);
void json_attachGPS(gpsInfo *gps);

#ifdef __cplusplus
}
#endif
#endif // !JSONPARSER_H
