idf_component_register(
  SRCS "esp_app_main.cpp"
  INCLUDE_DIRS "."
  PRIV_REQUIRES nvs_flash esp32_BNO08x
)
