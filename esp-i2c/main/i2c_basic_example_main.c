/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected
   over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include "esp_log.h"

#include "i2cManager.h"
static const char *TAG = "example";

void app_main(void) { i2cManager_start(); }
