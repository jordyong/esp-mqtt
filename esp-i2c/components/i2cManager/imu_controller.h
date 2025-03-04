#ifndef IMU_CONTROLLER_H
#define IMU_CONTROLLER_H

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

i2c_device_config_t get_imu_device_config();

void run_imu(i2c_master_dev_handle_t *dev_handle);

#endif // !IMU_CONTROLLER_H
