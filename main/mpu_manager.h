#ifndef MPU_MANAGER_H
#define MPU_MANAGER_H
#include <stdint.h>

// Fall detection state
typedef enum
{
    FALL_STATE_NORMAL,
    FALL_STATE_FREE_FALL,
    FALL_STATE_FALL_DETECTED
} fall_state_t;

// Sensor data structure
typedef struct
{
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float accel_magnitude;
    float gyro_magnitude;
    uint32_t timestamp;
} sensor_data_t;

void mpu_manager_start(void);
#endif
