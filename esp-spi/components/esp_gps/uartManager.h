#pragma once

#include "esp_err.h"

#define UART_NUM UART_NUM_1
#define TXD_PIN (CONFIG_ESP32_UART_TX)
#define RXD_PIN (CONFIG_ESP32_UART_RX)

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#ifdef __cplusplus
extern "C" {
#endif

esp_err_t uart_manager_start();
int sendData(const char *data);
void getGPSlatlong(double *lat, double *lng);

#ifdef __cplusplus
}
#endif
