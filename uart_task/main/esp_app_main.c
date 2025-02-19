#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "uartManager.h"

static void tx_task(void *arg) {
  static const char *TX_TASK_TAG = "TX_TASK";
  esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
  const char *input =
      "$GNGLL,0116.8066,N,10349.5124,E,153518.000,A,A4F\r\0\0\0";
  while (1) {
    sendData(input);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void test() {
  xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2,
              NULL);
}

void app_main(void) {
  uartManager_init();
  test();
}
