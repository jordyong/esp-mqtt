#include "i2cManager.h"
#include "imu_controller.h"

#include "esp_log.h"
static const char *TAG = "i2cManager";
/**
 * @brief i2c master initialization
 */
esp_err_t i2cManager_init(i2c_master_bus_handle_t *bus_handle,
                          i2c_master_dev_handle_t *dev_handle) {
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_MASTER_NUM,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

  i2c_device_config_t dev_config = get_imu_device_config();
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

  return ESP_OK;
}

void i2cManager_stop(i2c_master_bus_handle_t *bus_handle,
                     i2c_master_dev_handle_t *dev_handle) {
  ESP_ERROR_CHECK(i2c_master_bus_rm_device(*dev_handle));
  ESP_ERROR_CHECK(i2c_del_master_bus(*bus_handle));
  ESP_LOGI(TAG, "I2C de-initialized successfully");
}

void i2cManager_start() {
  i2c_master_bus_handle_t bus_handle;
  i2c_master_dev_handle_t dev_handle;
  ESP_ERROR_CHECK(i2cManager_init(&bus_handle, &dev_handle));
  ESP_LOGI(TAG, "I2C initialized successfully");

  run_imu(&dev_handle);

  /*i2cManager_stop(&bus_handle, &dev_handle);*/
}
