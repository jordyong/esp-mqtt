
#include "BNO08x.hpp"
#include "motors.h"
#include "uartManager.h"

#include "freertos/FreeRTOS.h"

#include "esp_log.h"
static const char *TAG = "main_tests";

void motor_test(BNO08x imu) {
  bno08x_euler_angle_t euler;
  while (1) {
    // block until new report is detected
    if (imu.data_available()) {

      if (imu.rpt.rv_game.has_new_data()) {
        euler = imu.rpt.rv_game.get_euler();
        ESP_LOGI(TAG, "Yaw: %.2f[deg]", euler.z);
      }
    }
    if (euler.z >= 10.0) {
      ESP_LOGI(TAG, "AAAA");
      setDirection(MOVERIGHT);
    } else if (euler.z <= -10.0) {
      ESP_LOGI(TAG, "BBBB");
      setDirection(MOVELEFT);
    } else {
      ESP_LOGI(TAG, "All good");
      setDirection(MOVESTOP);
    }
  }
}

void tx_task(void *arg) {
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

void gps_test() {
  xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2,
              NULL);
}
