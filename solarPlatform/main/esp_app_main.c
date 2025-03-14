
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

static void app_init(void) {
  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes",
           esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

  ESP_ERROR_CHECK(nvs_flash_init());
}

static void app_start() {
  ESP_ERROR_CHECK(wifi_manager_start());
  ESP_ERROR_CHECK(mqtt_manager_start());
  ESP_ERROR_CHECK(uart_manager_start());
}

static void tx_task(void *arg) {
  static const char *TX_TASK_TAG = "TX_TASK";
  esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
  const char *input =
      "{\n\"serial_type\":\"GPS\",\n\t\"data\":{\n\t\"lng\":103.003,"
      "\n\t\"lat\":1.002}\n}";
  while (1) {
    sendData(input);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

static void test() {
  xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2,
              NULL);
}

void app_main(void) {
  app_init();
  app_start();

  test();
}
