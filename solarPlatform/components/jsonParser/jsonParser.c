#include "gpsParser.h"
#include "models.h"
#include <cJSON.h>
#include <string.h>

#include "mqttManager.h"
#include "uartManager.h"

#include "esp_log.h"
static const char *TAG = "jsonParser";

// Analyse the Uart json and extract the data object
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
    mqtt_manager_publish_json("devices/status/gps", cJSON_Print(data), 0, 0);
  } else if (strcmp(serial_type_value, "Power") == 0) {
    cJSON_AddStringToObject(data, "device_ID", get_espID());
    mqtt_manager_publish_json("devices/status/power", cJSON_Print(data), 0, 0);
  } else {
    ESP_LOGI(TAG, "Unknown serial type: '%s'", serial_type_value);
  }

  cJSON_Delete(root);
}

// Take the json object from mqtt and pass it to uart
void json_mqttParser(const char *c) {
  cJSON *data = cJSON_Parse(c);
  if (data == NULL) {
    ESP_LOGI(TAG, "Not a valid JSON message");
    return;
  }
  cJSON_DeleteItemFromObject(data, "device_ID");

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "serial_type", "CMD");
  cJSON_AddItemToObject(root, "data", data);

  sendData(cJSON_Print(root));
}
