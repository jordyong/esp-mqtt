#include "freertos/idf_additions.h"

#include "gpsParser.h"
#include "models.h"
#include <cJSON.h>
#include <string.h>

#include "mqttManager.h"
#include "uartManager.h"

#include "esp_log.h"
static const char *TAG = "jsonParser";

static gpsInfo *_gps = NULL;

void json_attachGPS(gpsInfo *gps) {
  _gps = gps;
  return;
}

void json_mqttParser(const char *c) {
  cJSON *data = cJSON_Parse(c);
  if (data == NULL) {
    ESP_LOGI(TAG, "Not a valid JSON message");
    return;
  }

  cJSON *longitude = cJSON_GetObjectItem(data, "longitude");
  cJSON *latitude = cJSON_GetObjectItem(data, "latitude");

  ESP_LOGI(TAG, "Received GPS CMD: [%.6f, %.6f]", latitude->valuedouble,
           longitude->valuedouble);

  _gps->lng = longitude->valuedouble;
  _gps->lat = latitude->valuedouble;
  _gps->state = 1;
  ESP_LOGI(TAG, "Set movement\n");
}

void json_GPStoMQTT(float latitude, float longitude) {
  cJSON *data = cJSON_CreateObject();
  cJSON_AddStringToObject(data, "device_ID", get_espID());

  cJSON_AddNumberToObject(data, "longitude", longitude);
  cJSON_AddNumberToObject(data, "latitude", latitude);

  mqtt_manager_publish_json("devices/status/gps", cJSON_Print(data), 0, 0);
}
