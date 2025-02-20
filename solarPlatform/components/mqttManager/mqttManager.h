#pragma once
#include "esp_err.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*mqtt_message_callback_t)(const char *topic, const char *message,
                                        size_t message_len);

char *get_espID(void);
esp_err_t mqtt_manager_start(void);
esp_err_t mqtt_manager_stop(void);

esp_err_t mqtt_manager_publish_json(const char *topic, const char *json_msg,
                                    int qos, int retain);
esp_err_t mqtt_manager_subscribe(const char *topic, int qos,
                                 mqtt_message_callback_t callback);
#ifdef __cplusplus
}
#endif
