#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t uart_manager_start();
int sendData(const char *data);

#ifdef __cplusplus
}
#endif
