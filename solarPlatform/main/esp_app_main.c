
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"

#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "mqttManager.h"
#include "uartManager.h"
#include "wifiManager.h"

static const char *TAG = "Solar Platform";

void app_main(void) {
  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes",
           esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

  ESP_ERROR_CHECK(nvs_flash_init());

  ESP_ERROR_CHECK(wifi_manager_start());
  ESP_ERROR_CHECK(mqtt_manager_start());
  ESP_ERROR_CHECK(uartManager_init());
  mqtt_manager_publish_json("test", "test", 1, 0, 0);
}
