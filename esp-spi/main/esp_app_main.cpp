#include <cstdlib>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "BNO08x.hpp"
#include "jsonParser.h"
#include "models.h"
#include "motors.h"
#include "mqttManager.h"
#include "uartManager.h"
#include "wifiManager.h"

#include "esp_log.h"
static const char *TAG = "spi_test";
static BNO08x imu;

void imu_init(void) {
  ESP_LOGI(TAG, "Running spi_tests");
  // initialize imu
  if (!imu.initialize()) {
    ESP_LOGE(TAG, "Init failure, returning from main.");
    return;
  }

  // enable game rotation vector and calibrated gyro reports
  imu.rpt.rv_game.enable(100000UL); // 100,000us == 100ms report interval
  // see BNO08x::bno08x_reports_t for all possible reports to enable
}

gpsInfo _gps = {0, 0.0, 0.0};

void gps_move_task(void *args) {
  bno08x_euler_angle_t euler;
  while (1) {
    switch (_gps.state) {
    case 1: {

      double lon1 = 0.0, lon2 = _gps.lng; // jsondata.lng
      double lat1 = 0.0, lat2 = _gps.lat; // jsondata.lat

      getGPSlatlong(&lat1, &lon1);

      if (imu.rpt.rv_game.has_new_data()) {
        euler = imu.rpt.rv_game.get_euler();
      }
      double currHeading = euler.z;
      // Convert latitude to radians before using cos()
      double lat1_rad = lat1 * (M_PI / 180.0);
      double lat2_rad = lat2 * (M_PI / 180.0);

      // Convert GPS differences to meters
      double dLon = (lon2 - lon1) * (M_PI / 180.0);
      double dx = (lon2 - lon1) * cos(lat1_rad) * 111320.0;
      double dy = (lat2 - lat1) * 110540.0;
      double x = sin(dLon) * cos(lat2_rad);
      double y = cos(lat1_rad) * sin(lat2_rad) -
                 sin(lat1_rad) * cos(lat2_rad) * cos(dLon);
      double bearing = atan2(x, y) * (180.0 / M_PI);

      // Compute distance using Pythagorean theorem
      double distance = sqrt(dx * dx + dy * dy);

      // Compute target heading
      double turnangle = bearing + currHeading;
      // if (turnangle > 180) turnangle -= 360;
      // else if (turnangle < -180) turnangle += 360;

      if (turnangle > 3)
        setDirection(MOVERIGHT);
      else if (turnangle < -3)
        setDirection(MOVELEFT);
      else if (distance > 5) {
        setDirection(MOVEFRONT);
        _gps.state = 0;
      } else {
        setDirection(MOVESTOP);
      }

      ESP_LOGI(TAG, "Distance :%.2f", distance);
      ESP_LOGI(TAG, "Turnangle :%.2f", turnangle);

      break;
    }
    default: {
      ESP_LOGI(TAG, "IDLE");
      break;
    }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete(NULL);
}

static void app_init() {
  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes",
           esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

  motorsInit();
  imu_init();

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(wifi_manager_start());
  ESP_ERROR_CHECK(mqtt_manager_start());
  ESP_ERROR_CHECK(uart_manager_start());

  xTaskCreate(gps_move_task, "gps_move_task", 2048, NULL, 5, NULL);
  json_attachGPS(&_gps);
}

extern "C" void app_main(void) {
  app_init();

  /*gps_test();*/
}
