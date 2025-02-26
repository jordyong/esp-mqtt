#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "jsonParser.h"
#include "uartManager.h"

#include "esp_log.h"
static const char *TAG = "uartManager";

#define EX_UART_NUM UART_NUM_1
#define TXD_PIN (GPIO_NUM_32)
#define RXD_PIN (GPIO_NUM_33)

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart_queue;

int readData(const int len);

void uart_event_task(void *pvParameters) {
  uart_event_t event;
  for (;;) {
    // Waiting for UART event.
    if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
      ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
      switch (event.type) {
      // Event of UART receiving data
      /*We'd better handler data event fast, there would be much more data
      events than other types of events. If we take too much time on data event,
      the queue might be full.*/
      case UART_DATA:
        ESP_LOGI(TAG, "[UART DATA]: %zu", event.size);
        ESP_LOGI(TAG, "[DATA EVT]:");
        readData(event.size);
        break;
      // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control
        // for your application. The ISR has already reset the rx FIFO, As an
        // example, we directly flush the rx buffer here in order to read more
        // data.
        uart_flush_input(EX_UART_NUM);
        xQueueReset(uart_queue);
        break;
      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full");
        // If buffer full happened, you should consider increasing your buffer
        // size As an example, we directly flush the rx buffer here in order to
        // read more data.
        uart_flush_input(EX_UART_NUM);
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

int sendData(const char *data) {
  const int len = strlen(data);
  const int txBytes = uart_write_bytes(EX_UART_NUM, data, len);
  ESP_LOGI(TAG, "Wrote %d bytes", txBytes);
  return txBytes;
}

int readData(const int len) {
  // read from UART buffer
  uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
  bzero(dtmp, RD_BUF_SIZE);
  const int rxBytes = uart_read_bytes(EX_UART_NUM, dtmp, len, portMAX_DELAY);
  ESP_LOGI(TAG, "Read '%s', [%d bytes]\n", dtmp, rxBytes);
  // parse json data
  json_uartParser((char *)dtmp);

  // clean up
  free(dtmp);
  dtmp = NULL;
  return rxBytes;
}

static esp_err_t uart_init() {
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  // Install UART driver, and get the queue
  uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue,
                      0);
  uart_param_config(EX_UART_NUM, &uart_config);

  // Set UART pins
  uart_set_pin(EX_UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
  return ESP_OK;
}

esp_err_t uart_manager_start(void) {
  ESP_LOGI(TAG, "Starting UART Manager...");
  esp_log_level_set(TAG, ESP_LOG_INFO);
  uart_init();

  // Create a task to handle UART event from ISR
  xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 12, NULL);
  return ESP_OK;
}
