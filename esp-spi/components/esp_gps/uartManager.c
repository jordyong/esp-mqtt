#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

/*#include "gps_reader.h"*/
#include "jsonParser.h"
#include "uartManager.h"

#include "esp_log.h"
static const char *TAG = "uartManager";

static QueueHandle_t uart_queue;

void uart_event_task(void *pvParameters) {
  uart_event_t event;
  for (;;) {
    // Waiting for UART event.
    if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
      ESP_LOGI(TAG, "uart[%d] event:", UART_NUM);
      switch (event.type) {
      // Event of UART receiving data
      /*We'd better handler data event fast, there would be much more data
      events than other types of events. If we take too much time on data event,
      the queue might be full.*/
      case UART_DATA:
        ESP_LOGI(TAG, "[UART DATA]: %zu", event.size);
        ESP_LOGI(TAG, "[DATA EVT]:");
        break;
      // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control
        // for your application. The ISR has already reset the rx FIFO, As an
        // example, we directly flush the rx buffer here in order to read more
        // data.
        uart_flush_input(UART_NUM);
        xQueueReset(uart_queue);
        break;
      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full");
        // If buffer full happened, you should consider increasing your buffer
        // size As an example, we directly flush the rx buffer here in order to
        // read more data.
        uart_flush_input(UART_NUM);
        xQueueReset(uart_queue);
        break;
      // Event of UART RX break detected
      case UART_BREAK:
        ESP_LOGI(TAG, "uart rx break");
        break;
      // Event of UART parity check error
      case UART_PARITY_ERR:
        ESP_LOGI(TAG, "uart parity error");
        break;
      // Event of UART frame error
      case UART_FRAME_ERR:
        ESP_LOGI(TAG, "uart frame error");
        break;
      // UART_PATTERN_DET
      default:
        ESP_LOGI(TAG, "uart event type: %d", event.type);
        break;
      }
    }
  }
  vTaskDelete(NULL);
}

typedef struct gps {
  float nmeaLat;
  char ns;
  float nmeaLong;
  char ew;
  float utc;
  char gllstats;

  double deciLat;
  double deciLong;
} GPS_data;

static GPS_data GPS;

float GPS_nmea_to_dec(float deg_coord, char nsew) {
  int degree = (int)(deg_coord / 100.0);  // Extract degrees correctly
  float minutes = fmod(deg_coord, 100.0); // Use fmod for precise minutes
  float dec_deg = minutes / 60.0;
  float decimal = degree + dec_deg;
  if (nsew == 'S' || nsew == 'W') { // return negative
    decimal *= -1.0;
  }
  return decimal;
}

void getGPSlatlong(double *lat, double *lng) {
  *lat = GPS.deciLat;
  *lng = GPS.deciLong;
  return;
}

void uart_rx_task(void *arg) {
  uint8_t *data = (uint8_t *)malloc(RD_BUF_SIZE);
  char line[256];     // Buffer to store a single line
  int line_index = 0; // Track position in line buffer

  while (1) {
    // Read available data from UART
    bzero(data, RD_BUF_SIZE);
    int len =
        uart_read_bytes(UART_NUM, data, RD_BUF_SIZE - 1, pdMS_TO_TICKS(1000));

    if (len > 0) {
      data[len] = '\0'; // Null-terminate the received data
      /*ESP_LOGI(TAG, "Raw: %s", data);*/

      // Process each character received
      for (int i = 0; i < len; i++) {
        if (data[i] == '\n') {     // End of a line
          line[line_index] = '\0'; // Null-terminate line
          // printf("Line: %s\n", line);  // Print full line

          // Check if the line contains "$GNGLL"
          if (strstr(line, "$GNGLL") != NULL) {
            if (sscanf(line, "$GNGLL,%f,%c,%f,%c,%f,%c", &GPS.nmeaLat, &GPS.ns,
                       &GPS.nmeaLong, &GPS.ew, &GPS.utc, &GPS.gllstats) >= 1) {
              GPS.deciLat = GPS_nmea_to_dec(GPS.nmeaLat, GPS.ns);
              GPS.deciLong = GPS_nmea_to_dec(GPS.nmeaLong, GPS.ew);
              ESP_LOGI(TAG, "GPS: [%.6f,%.6f]", GPS.deciLat, GPS.deciLong);
              json_GPStoMQTT(GPS.deciLat, GPS.deciLong);
            }
          }
          // Reset line buffer for next line
          line_index = 0;
        } else if (line_index < sizeof(line) - 1) {
          line[line_index++] = data[i]; // Add character to line buffer
        }
      }
    }
  }
  vTaskDelete(NULL);
}

int sendData(const char *data) {
  const int len = strlen(data);
  const int txBytes = uart_write_bytes(UART_NUM, data, len);
  ESP_LOGI(TAG, "Wrote %d bytes", txBytes);
  return txBytes;
}

void configure_uart(uart_port_t uart_num, int tx_pin, int rx_pin,
                    int baud_rate) {
  uart_config_t uart_config = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(
      uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
}

esp_err_t uart_manager_start(void) {
  ESP_LOGI(TAG, "Starting UART Manager...");
  esp_log_level_set(TAG, ESP_LOG_INFO);

  configure_uart(UART_NUM, TXD_PIN, RXD_PIN, 9600);
  // Create a task to handle UART event from ISR
  xTaskCreate(uart_rx_task, "uart_rx_task", 3072, NULL, 3, NULL);
  return ESP_OK;
}
