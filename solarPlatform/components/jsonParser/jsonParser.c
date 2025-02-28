#include "gpsParser.h"
#include "models.h"
#include <cJSON.h>
#include <string.h>

#include "mqttManager.h"
#include "uartManager.h"

#include "esp_log.h"
static const char *TAG = "jsonParser";

void json_uartParser(const char *c) {
  cJSON *root = cJSON_Parse(c);
  if (root == NULL) {
    ESP_LOGI(TAG, "Not a valid JSON message");
    return;
  }

  cJSON *serial_type = cJSON_GetObjectItem(root, "serial_type");
  if (serial_type == NULL) {
    ESP_LOGI(TAG, "'serial_type' field not found");
    return;
  }

  cJSON *data = cJSON_GetObjectItem(root, "data");
  if (data == NULL) {
    ESP_LOGI(TAG, "'data' field not found");
    return;
  }
  char *serial_type_value = serial_type->valuestring;

  if (strcmp(serial_type_value, "GPS") == 0) {
    cJSON_AddStringToObject(data, "device_ID", get_espID());

    cJSON *longitude = cJSON_GetObjectItem(data, "longitude");
    cJSON *latitude = cJSON_GetObjectItem(data, "latitude");

    longitude->valuedouble = ddmmIntoDD(longitude->valuedouble);
    latitude->valuedouble = ddmmIntoDD(latitude->valuedouble);

    mqtt_manager_publish_json("devices/status/gps", cJSON_Print(data), 0, 0);
  } else {
    ESP_LOGI(TAG, "Unknown serial type: '%s'", serial_type_value);
  }

  cJSON_Delete(root);
}

void json_mqttParser(const char *c) {
  cJSON *data = cJSON_Parse(c);
  if (data == NULL) {
    ESP_LOGI(TAG, "Not a valid JSON message");
    return;
  }
  cJSON_DeleteItemFromObject(data, "device_ID");

  cJSON *longitude = cJSON_GetObjectItem(data, "longitude");
  cJSON *latitude = cJSON_GetObjectItem(data, "latitude");

  longitude->valuedouble = DDToddmm(longitude->valuedouble);
  latitude->valuedouble = DDToddmm(latitude->valuedouble);

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "serial_type", "GPS");
  cJSON_AddItemToObject(root, "data", data);

  sendData(cJSON_Print(root));
}
