#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "mpu6050.h" // IMU sensor
#include <math.h>

static int16_t accel_x, accel_y, accel_z;
static int16_t gyro_x, gyro_y, gyro_z;
static float roll = 0, pitch = 0, yaw = 0;
int16_t accel_x_offset = 0;
int16_t accel_y_offset = 0;
int16_t accel_z_offset = 0;
int16_t gyro_x_offset = 0;
int16_t gyro_y_offset = 0;
int16_t gyro_z_offset = 0;

void calibrate_imu(void *arg) {
  long accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
  long gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  int sample_count = 1000; // Number of samples
  for (int i = 0; i < sample_count; i++) {
    if (mpu6050_read_raw(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y,
                         &gyro_z) == ESP_OK) {
      accel_x_sum += accel_x;
      accel_y_sum += accel_y;
      accel_z_sum += accel_z;
      gyro_x_sum += gyro_x;
      gyro_y_sum += gyro_y;
      gyro_z_sum += gyro_z;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
  accel_x_offset = accel_x_sum / sample_count;
  accel_y_offset = accel_y_sum / sample_count;
  accel_z_offset = (accel_z_sum / sample_count) -
                   16384; // 16384 is 1g in the default accelerometer scale
  gyro_x_offset = gyro_x_sum / sample_count;
  gyro_y_offset = gyro_y_sum / sample_count;
  gyro_z_offset = gyro_z_sum / sample_count;

  ESP_LOGI(TAG, "Calibration complete.");
  ESP_LOGI(TAG, "Accel offsets: X=%d, Y=%d, Z=%d", accel_x_offset,
           accel_y_offset, accel_z_offset);
  ESP_LOGI(TAG, "Gyro offsets: X=%d, Y=%d, Z=%d", gyro_x_offset, gyro_y_offset,
           gyro_z_offset);
}

static float imu_invSqrt(float x) {
  /* Fast inverse square-root See:
   * http://en.wikipedia.org/wiki/Fast_inverse_square_root */

  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;

  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));

  return y;
}

// Kalman Filter Variables
static float bias = 0.0;                 // Bias estimate
static float P[2][2] = {{1, 0}, {0, 1}}; // Error covariance matrix
static float Q_angle = 0.001;            // Process noise variance for angle
static float Q_bias = 0.003;             // Process noise variance for bias
static float R_measure = 0.03;           // Measurement noise variance
static float kalman_filter(float newAngle, float newRate, float dt) {
  // Prediction Step
  yaw += dt * (newRate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Update Step
  float S = P[0][0] + R_measure; // Estimate error
  float K[2];                    // Kalman Gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - yaw; // Measurement residual
  yaw += K[0] * y;
  bias += K[1] * y;

  // Update error covariance matrix
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return yaw;
}

void rpy_complimentary_filter(int ax, int ay, int az, int gx, int gy, int gz,
                              int64_t dt) {
  // Convert raw accelerometer values to g's
  float ax_g = (float)ax / 16384.0f;
  float az_g = (float)ax / 16384.0f;
  float ay_g = (float)ax / 16384.0f;

  // Convert raw gyroscope values to degrees/second
  float g_roll = (float)gx / 131.0f;
  float g_pitch = (float)gy / 131.0f;
  float g_yaw = (float)gz / 131.0f;

  // Calculate accelerometer roll and pitch
  float a_roll = atan2f(ay_g, az_g) * 180 / M_PI;
  float a_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180 / M_PI;

  dt = dt / 1000000.0f;

  roll = 0.98f * (roll + g_roll * dt) + 0.02f * a_roll;
  pitch = 0.98f * (pitch + g_pitch * dt) + 0.02f * a_pitch;
  yaw += g_yaw * dt;
}

void read_imu_task(void *arg) {
  int64_t dt, current_time, last_time = 0;
  while (1) {
    if (mpu6050_read_raw(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y,
                         &gyro_z) == ESP_OK) {
      ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d", accel_x - accel_x_offset,
               accel_y - accel_y_offset, accel_z - accel_z_offset);
      ESP_LOGI(TAG, "Gyro: X=%d, Y=%d, Z=%d", gyro_x - gyro_x_offset,
               gyro_y - gyro_y_offset, gyro_z - gyro_z_offset);

      current_time = esp_timer_get_time();
      dt = current_time - last_time;
      last_time = current_time;

      rpy_complimentary_filter(accel_x, accel_y, accel_z, gyro_x, gyro_y,
                               gyro_z, dt);

      ESP_LOGI(TAG, "Roll: %f, Pitch %f, Yaw: %.2f", roll, pitch, yaw);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void run_imu(i2c_master_dev_handle_t *dev_handle) {
  /* Read the MPU9250 WHO_AM_I register, on power up the register should have
   * the value 0x71 */
  mpu6050_init(dev_handle);
  mpu6050_wakeup(_dev_handle);
  calibrate_imu(NULL);
  xTaskCreate(read_imu_task, "read_imu_task", 1024 * 2, NULL,
              configMAX_PRIORITIES - 2, NULL);
}

i2c_device_config_t get_imu_device_config() {
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MPU6050_SENSOR_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };
  return dev_config;
}
