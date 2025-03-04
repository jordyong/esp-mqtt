#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "i2cManager.h"

#define MPU6050_SENSOR_ADDR 0x68

#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40

#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48

#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C

#define MPU6050_WHO_AM_I_REG_ADDR 0x75

#define MPU6050_RESET_BIT 7

#include "esp_log.h"
static const char *TAG = "MPU6050";
static i2c_master_dev_handle_t _dev_handle;
/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle,
                                uint8_t reg_addr, uint8_t *data, size_t len) {
  return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len,
                                     I2C_MASTER_TIMEOUT_MS /
                                         portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle,
                                      uint8_t reg_addr, uint8_t data) {
  uint8_t write_buf[2] = {reg_addr, data};
  return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf),
                             I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(i2c_master_dev_handle_t *dev_handle) {
  _dev_handle = *dev_handle;

  uint8_t data[2];
  ESP_ERROR_CHECK(
      mpu6050_register_read(_dev_handle, MPU6050_WHO_AM_I_REG_ADDR, data, 1));
  ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

  return ESP_OK;
}

esp_err_t mpu6050_wakeup(i2c_master_dev_handle_t dev_handle) {
  return mpu6050_register_write_byte(dev_handle, MPU6050_RA_PWR_MGMT_1, 1);
}

esp_err_t mpu6050_read_accel(int16_t *accel_x, int16_t *accel_y,
                             int16_t *accel_z) {
  uint8_t data[6];
  esp_err_t ret =
      mpu6050_register_read(_dev_handle, MPU6050_RA_ACCEL_XOUT_H, data, 6);
  if (ret == ESP_OK) {
    *accel_x = (data[0] << 8) | data[1];
    *accel_y = (data[2] << 8) | data[3];
    *accel_z = (data[4] << 8) | data[5];
  }
  return ret;
}

esp_err_t mpu6050_read_gyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
  uint8_t data[6];
  esp_err_t ret =
      mpu6050_register_read(_dev_handle, MPU6050_RA_GYRO_XOUT_H, data, 6);
  if (ret == ESP_OK) {
    *gyro_x = (data[0] << 8) | data[1];
    *gyro_y = (data[2] << 8) | data[3];
    *gyro_z = (data[4] << 8) | data[5];
  }
  return ret;
}

esp_err_t mpu6050_read_raw(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                           int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
  uint8_t data[14];
  esp_err_t ret =
      mpu6050_register_read(_dev_handle, MPU6050_RA_ACCEL_XOUT_H, data, 14);
  if (ret == ESP_OK) {
    *accel_x = (data[0] << 8) | data[1];
    *accel_y = (data[2] << 8) | data[3];
    *accel_z = (data[4] << 8) | data[5];
    *gyro_x = (data[8] << 8) | data[9];
    *gyro_y = (data[10] << 8) | data[11];
    *gyro_z = (data[12] << 8) | data[13];
  }
  return ret;
}

#endif
