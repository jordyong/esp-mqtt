#include "models.h"
#include <cJSON.h>
#include <string.h>

#include "mqttManager.h"

#include "esp_log.h"
static const char *TAG = "jsonParser";

void json_uartParser(const char *c) {
  cJSON *root = cJSON_Parse(c);

  cJSON *serial_type = cJSON_GetObjectItem(root, "serial_type");
  char *serial_type_value = serial_type->valuestring;

  if (strcmp(serial_type_value, "gps") == 0) {
    cJSON_AddStringToObject(root, "device_ID", get_espID());
    mqtt_manager_publish_json("devices/status/gps", cJSON_Print(root), 0, 0);
  } else {
    ESP_LOGI(TAG, "Unknown serial type: '%s'", serial_type_value);
  }

  cJSON_Delete(root);
}

void json_mqttParser(const char *c) { ESP_LOGI(TAG, "received mqtt: '%s'", c); }
