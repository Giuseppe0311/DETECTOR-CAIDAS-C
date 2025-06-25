//
// Created by barre on 6/25/2025.
//

#ifndef MPU_MANAGER_H
#define MPU_MANAGER_H
#include "esp_err.h"

void mpu6050_init(void);
esp_err_t i2c_master_init(void);
void mpu6050_read_raw(int16_t* accel, int16_t* gyro);
void mpu_manager_start(void);
#endif
