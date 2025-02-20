
#include <cJSON.h>
#include <stdint.h>
// c std files
#include <stdio.h>
#include <string.h>
// ESP systems
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
// ESP-idf components
#include "mqtt_client.h"
// User-defined
#include "jsonParser.h"
#include "mqttManager.h"

static const char *TAG = "MQTT Manager";
static char *clientID;
static esp_mqtt_client_handle_t client;

static const char *GPS_TOPIC = "devices/gps";

typedef struct {
  const char *topic;
  mqtt_message_callback_t callback;
} CallBackObj;

#define MAX_CALLBACKS 10
CallBackObj callbacks[MAX_CALLBACKS];

static int register_callback(const char *topic,
                             mqtt_message_callback_t callback) {
  for (int i = 0; i < MAX_CALLBACKS; i++) {
    if (callbacks[i].topic == NULL) {
      callbacks[i].topic = strdup(topic);
      callbacks[i].callback = callback;
    }
  }
  return -1;
}

static mqtt_message_callback_t get_callback(const char *topic) {
  for (int i = 0; i < MAX_CALLBACKS; i++) {
    if (callbacks[i].topic != NULL && strcmp(callbacks[i].topic, topic)) {
      return callbacks[i].callback;
    }
  }
  return NULL;
}

static char *create_json_string(const char *key, const int value) {
  cJSON *root = cJSON_CreateObject();
  cJSON_AddNumberToObject(root, key, value);
  char *json_string = cJSON_Print(root);
  cJSON_Delete(root);
  return json_string;
}

static void log_error_if_nonzero(const char *message, int error_code) {
  if (error_code != 0) {
    ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
  }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG, "Event dispatched from mqtt manager");
  esp_mqtt_event_handle_t event = event_data;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    esp_mqtt_client_publish(client, "devices/connected",
                            create_json_string(clientID, 1), 0, 2, 1);
    mqtt_manager_subscribe("cmd/#", 0, NULL);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);

    /*json_mqttParser(event->data);*/
    mqtt_message_callback_t callback = get_callback(event->topic);

    if (callback != NULL) {
      callback(event->topic, event->data, event->data_len);
    }
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      log_error_if_nonzero("reported from esp-tls",
                           event->error_handle->esp_tls_last_esp_err);
      log_error_if_nonzero("reported from tls stack",
                           event->error_handle->esp_tls_stack_err);
      log_error_if_nonzero("captured as transport's socket errno",
                           event->error_handle->esp_transport_sock_errno);
      ESP_LOGI(TAG, "Last errno string (%s)",
               strerror(event->error_handle->esp_transport_sock_errno));
    }
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

char *get_espID(void) {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);

  size_t nbytes =
      snprintf(NULL, 0, "ESP32_%02X%02X%02X", mac[3], mac[4], mac[5]) + 1;
  char *espID = calloc(1, nbytes);
  snprintf(espID, nbytes, "ESP32_%02X%02X%02X", mac[3], mac[4], mac[5]);

  return espID;
}

esp_err_t mqtt_manager_start(void) {
  ESP_LOGI(TAG, "Starting MQTT Manager...");
  clientID = get_espID();

  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = CONFIG_BROKER_URL,
      .credentials.client_id = clientID,
      .session.keepalive = 60,
      .session.last_will =
          {
              .topic = "devices/disconnected",
              .msg = create_json_string(clientID, 0),
              .msg_len = 0,
              .qos = 2,
              .retain = 1,
          },
  };

  client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,
                                 NULL);
  esp_mqtt_client_start(client);

  return ESP_OK;
}

esp_err_t mqtt_manager_stop(void) { return ESP_OK; }

esp_err_t mqtt_manager_publish_json(const char *topic, const char *json_msg,
                                    int qos, int retain) {

  int msg_id = esp_mqtt_client_publish(client, topic, json_msg, 0, qos, retain);
  if (msg_id < 0) {
    ESP_LOGE(TAG, "Failed to publish to topic :%s", topic);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Sent publish successful, msg_id=%d", msg_id);
  return ESP_OK;
}

esp_err_t mqtt_manager_subscribe(const char *topic, int qos,
                                 mqtt_message_callback_t callback) {
  size_t nbytes = snprintf(NULL, 0, "devices/%s/%s", clientID, topic) + 1;
  char *espID_topic = calloc(1, nbytes);
  snprintf(espID_topic, nbytes, "devices/%s/%s", clientID, topic);

  int msg_id = esp_mqtt_client_subscribe(client, espID_topic, qos);
  if (msg_id < 0) {
    ESP_LOGE(TAG, "Failed to subscribe to topic: %s", espID_topic);
    return ESP_FAIL;
  }

  if (callback != NULL) {
    if (register_callback(espID_topic, callback) < 0) {
      ESP_LOGE(TAG, "Failed to register callback, max callback reached");
      return ESP_FAIL;
    }
  }

  ESP_LOGI(TAG, "Sent subscribe successful, msg_id=%d", msg_id);
  return ESP_OK;
}
