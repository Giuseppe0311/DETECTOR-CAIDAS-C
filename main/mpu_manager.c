// #include "mpu_manager.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_SCL_IO          22
#define I2C_MASTER_FREQ_HZ         400000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

#define MPU6050_ADDR               0x68
#define PWR_MGMT_1_REG             0x6B
#define SMPLRT_DIV_REG             0x19
#define CONFIG_REG                 0x1A
#define GYRO_CONFIG_REG            0x1B
#define ACCEL_CONFIG_REG           0x1C
#define ACCEL_XOUT_H               0x3B
#define GYRO_XOUT_H                0x43

// Fall detection constants
#define HISTORY_SIZE               8
#define FREE_FALL_MIN_SAMPLES      3
#define CALIBRATION_SAMPLES        80
#define ORIENTATION_SAMPLES        30

static const char* TAG = "FALL_DETECT";

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

// Fall detection variables
static float gravity_baseline = 1.0f;
static float y_baseline = 0.5f;
static float initial_roll = 0.0f;
static float initial_pitch = 0.0f;

static fall_state_t fall_state = FALL_STATE_NORMAL;
static uint32_t free_fall_start = 0;
static uint32_t last_alert_time = 0;
static float last_confidence = 0.0f;

// Circular buffers for history
static float accel_history[HISTORY_SIZE];
static float gyro_history[HISTORY_SIZE];
static int history_index = 0;
static bool history_full = false;

// Thresholds (optimized for arm detection)
static float free_fall_threshold = 0.65f;
static float free_fall_threshold_y = 0.3f;
static float gyro_threshold = 250.0f;
static float impact_threshold = 1.8f;
static float impact_threshold_y = 2.5f;
static float orientation_change_thresh = 25.0f;
static uint32_t min_alert_interval = 4000; // ms
static uint32_t free_fall_duration = 150; // ms

// Derived thresholds
static float axis_free_fall_thresh = 0.25f;
static float axis_free_fall_thresh_y = 0.15f;
static float axis_impact_thresh = 1.5f;
static float axis_impact_thresh_y = 2.2f;

// Function declarations
static esp_err_t i2c_master_init(void);
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data);
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t* buf, size_t len);
static void mpu6050_init(void);
static bool mpu6050_read_raw(int16_t* accel, int16_t* gyro);
static void convert_raw_to_physical(int16_t* raw_accel, int16_t* raw_gyro, sensor_data_t* data);
static bool calibrate_gravity(void);
static bool record_initial_orientation(void);
static void update_history(float a_mag, float g_mag);
static bool detect_free_fall_window(void);
static bool detect_axis_impact(sensor_data_t* data);
static void compute_orientation(sensor_data_t* data, float* roll, float* pitch);
static void analyze_fall_software(sensor_data_t* data);
static void fall_detection_task(void* arg);

// Initialize I2C as master
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Write single byte to MPU6050 register
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                      (uint8_t[]){reg, data}, 2, pdMS_TO_TICKS(1000));
}

// Read multiple consecutive bytes from MPU6050
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t* buf, size_t len)
{
    // First send start address
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR,
        &reg, 1, pdMS_TO_TICKS(1000)));
    // Then read data
    return i2c_master_read_from_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                       buf, len, pdMS_TO_TICKS(1000));
}

// Configure MPU6050
static void mpu6050_init(void)
{
    // Wake up (clear sleep bit)
    mpu6050_write_reg(PWR_MGMT_1_REG, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Sample rate = Gyro output rate / (1 + SMPLRT_DIV)
    mpu6050_write_reg(SMPLRT_DIV_REG, 0x09); // 1kHz/(1+9) = 100Hz

    // DLPF config
    mpu6050_write_reg(CONFIG_REG, 0x1A);

    // Gyro full scale ±250°/s
    mpu6050_write_reg(GYRO_CONFIG_REG, 0x00);

    // Accel full scale ±2g
    mpu6050_write_reg(ACCEL_CONFIG_REG, 0x00);

    ESP_LOGI(TAG, "MPU6050 initialized");
}

// Read raw data and return success/failure
static bool mpu6050_read_raw(int16_t* accel, int16_t* gyro)
{
    uint8_t buf[14];
    // Read accelerometer (6), temperature (2), gyroscope (6)
    if (mpu6050_read_regs(ACCEL_XOUT_H, buf, 14) == ESP_OK)
    {
        accel[0] = (buf[0] << 8) | buf[1];
        accel[1] = (buf[2] << 8) | buf[3];
        accel[2] = (buf[4] << 8) | buf[5];
        gyro[0] = (buf[8] << 8) | buf[9];
        gyro[1] = (buf[10] << 8) | buf[11];
        gyro[2] = (buf[12] << 8) | buf[13];
        return true;
    }
    else
    {
        ESP_LOGW(TAG, "Error reading MPU6050 data");
        return false;
    }
}

// Convert raw values to physical units
static void convert_raw_to_physical(int16_t* raw_accel, int16_t* raw_gyro, sensor_data_t* data)
{
    // Convert accelerometer (±2g range, 16384 LSB/g)
    data->accel_x = raw_accel[0] / 16384.0f;
    data->accel_y = raw_accel[1] / 16384.0f;
    data->accel_z = raw_accel[2] / 16384.0f;

    // Convert gyroscope (±250°/s range, 131 LSB/°/s)
    data->gyro_x = raw_gyro[0] / 131.0f;
    data->gyro_y = raw_gyro[1] / 131.0f;
    data->gyro_z = raw_gyro[2] / 131.0f;

    // Calculate magnitudes
    data->accel_magnitude = sqrtf(data->accel_x * data->accel_x +
        data->accel_y * data->accel_y +
        data->accel_z * data->accel_z);
    data->gyro_magnitude = sqrtf(data->gyro_x * data->gyro_x +
        data->gyro_y * data->gyro_y +
        data->gyro_z * data->gyro_z);

    data->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// Calibrate gravity baseline
static bool calibrate_gravity(void)
{
    ESP_LOGI(TAG, "Keep arm relaxed for calibration...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    float total = 0;
    float sum_y = 0;
    int valid_samples = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        int16_t raw_accel[3], raw_gyro[3];
        sensor_data_t data;

        if (mpu6050_read_raw(raw_accel, raw_gyro))
        {
            convert_raw_to_physical(raw_accel, raw_gyro, &data);

            // Only count valid samples
            if (data.accel_magnitude > 0.5f && data.accel_magnitude < 2.0f)
            {
                total += data.accel_magnitude;
                sum_y += fabsf(data.accel_y);
                valid_samples++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(15));

        if (i % 20 == 0)
        {
            ESP_LOGI(TAG, "Calibrating... %d%%", (i / 20 + 1) * 25);
        }
    }

    if (valid_samples >= CALIBRATION_SAMPLES * 0.7f)
    {
        gravity_baseline = total / valid_samples;
        y_baseline = sum_y / valid_samples;

        // Update derived thresholds
        axis_free_fall_thresh = 0.25f * gravity_baseline;
        axis_free_fall_thresh_y = free_fall_threshold_y * gravity_baseline;
        axis_impact_thresh = 1.5f * gravity_baseline;
        axis_impact_thresh_y = impact_threshold_y * gravity_baseline;

        ESP_LOGI(TAG, "Calibration complete: %.2fg", gravity_baseline);
        return true;
    }
    else
    {
        ESP_LOGW(TAG, "Calibration failed, using defaults");
        return false;
    }
}

// Record initial orientation
static bool record_initial_orientation(void)
{
    float sum_roll = 0;
    float sum_pitch = 0;
    int valid_samples = 0;

    for (int i = 0; i < ORIENTATION_SAMPLES; i++)
    {
        int16_t raw_accel[3], raw_gyro[3];
        sensor_data_t data;
        float roll, pitch;

        if (mpu6050_read_raw(raw_accel, raw_gyro))
        {
            convert_raw_to_physical(raw_accel, raw_gyro, &data);
            compute_orientation(&data, &roll, &pitch);

            // Only count valid samples
            if (roll >= -180 && roll <= 180 && pitch >= -180 && pitch <= 180)
            {
                sum_roll += roll;
                sum_pitch += pitch;
                valid_samples++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (valid_samples > 0)
    {
        initial_roll = sum_roll / valid_samples;
        initial_pitch = sum_pitch / valid_samples;
        ESP_LOGI(TAG, "Initial orientation: roll=%.1f°, pitch=%.1f°", initial_roll, initial_pitch);
        return true;
    }
    else
    {
        ESP_LOGW(TAG, "Could not determine initial orientation");
        return false;
    }
}

// Update circular history buffer
static void update_history(float a_mag, float g_mag)
{
    accel_history[history_index] = a_mag;
    gyro_history[history_index] = g_mag;
    history_index = (history_index + 1) % HISTORY_SIZE;
    if (!history_full && history_index == 0)
    {
        history_full = true;
    }
}

// Detect free fall window
static bool detect_free_fall_window(void)
{
    if (!history_full)
    {
        int count = 0;
        for (int i = 0; i < HISTORY_SIZE; i++)
        {
            if (accel_history[i] > 0) count++;
        }
        if (count < FREE_FALL_MIN_SAMPLES) return false;
    }

    // Get last samples from circular buffer
    bool mag_ok = true;
    bool gyro_ok = true;

    for (int i = 0; i < FREE_FALL_MIN_SAMPLES; i++)
    {
        int idx = (history_index - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
        if (accel_history[idx] >= gravity_baseline * free_fall_threshold)
        {
            mag_ok = false;
        }
        if (gyro_history[idx] >= gyro_threshold)
        {
            gyro_ok = false;
        }
    }

    return mag_ok && gyro_ok;
}

// Detect axis impact
static bool detect_axis_impact(sensor_data_t* data)
{
    // Strong impact on Y axis (dominant) - higher threshold
    bool y_impact = fabsf(data->accel_y) > axis_impact_thresh_y;

    // Impact on other axes - lower threshold
    bool xz_impact = (fabsf(data->accel_x) > axis_impact_thresh ||
        fabsf(data->accel_z) > axis_impact_thresh);

    return y_impact || xz_impact;
}

// Compute orientation
static void compute_orientation(sensor_data_t* data, float* roll, float* pitch)
{
    *roll = atan2f(data->accel_y, data->accel_z) * 180.0f / M_PI;
    *pitch = atan2f(-data->accel_x, sqrtf(data->accel_y * data->accel_y +
                        data->accel_z * data->accel_z)) * 180.0f / M_PI;
}

// Main fall analysis function
static void analyze_fall_software(sensor_data_t* data)
{
    update_history(data->accel_magnitude, data->gyro_magnitude);

    switch (fall_state)
    {
    case FALL_STATE_NORMAL:
        if (detect_free_fall_window())
        {
            fall_state = FALL_STATE_FREE_FALL;
            free_fall_start = data->timestamp;
            ESP_LOGI(TAG, "Free fall detected");
        }
        break;

    case FALL_STATE_FREE_FALL:
        {
            uint32_t dt = data->timestamp - free_fall_start;

            // Detect impact
            bool magnitude_impact = (data->accel_magnitude > impact_threshold &&
                dt >= free_fall_duration);
            bool axis_impact = detect_axis_impact(data);
            bool impact = magnitude_impact || axis_impact;

            if (impact)
            {
                uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (current_time - last_alert_time < min_alert_interval)
                {
                    fall_state = FALL_STATE_NORMAL;
                    break;
                }

                float roll, pitch;
                compute_orientation(data, &roll, &pitch);
                float delta = fabsf(roll - initial_roll) + fabsf(pitch - initial_pitch);

                if (delta >= orientation_change_thresh)
                {
                    fall_state = FALL_STATE_FALL_DETECTED;
                    last_confidence = 80.0f;
                    last_alert_time = current_time;
                    ESP_LOGW(TAG, "*** CAIDA DETECTADA *** Confidence: %.1f%%", last_confidence);
                }
                else
                {
                    fall_state = FALL_STATE_NORMAL;
                }
            }
            else if (dt > (free_fall_duration * 4)) // Very long free fall
            {
                float roll, pitch;
                compute_orientation(data, &roll, &pitch);
                float delta = fabsf(roll - initial_roll) + fabsf(pitch - initial_pitch);

                if (delta >= orientation_change_thresh * 0.6f)
                {
                    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (current_time - last_alert_time >= min_alert_interval)
                    {
                        fall_state = FALL_STATE_FALL_DETECTED;
                        last_confidence = 65.0f;
                        last_alert_time = current_time;
                        ESP_LOGW(TAG, "*** CAIDA DETECTADA *** (Long fall) Confidence: %.1f%%", last_confidence);
                    }
                    else
                    {
                        fall_state = FALL_STATE_NORMAL;
                    }
                }
                else
                {
                    fall_state = FALL_STATE_NORMAL;
                }
            }
            break;
        }

    case FALL_STATE_FALL_DETECTED:
        {
            uint32_t dt_since_detection = data->timestamp - last_alert_time;
            if (dt_since_detection > 8000) // Auto-reset after 8 seconds
            {
                fall_state = FALL_STATE_NORMAL;
                last_confidence = 0.0f;
                ESP_LOGI(TAG, "Fall state auto-reset");
                break;
            }

            // Stability conditions to return to normal
            bool accel_stable = fabsf(data->accel_magnitude - gravity_baseline) < 0.3f;
            bool gyro_stable = data->gyro_magnitude < 80.0f;
            bool y_stable = fabsf(data->accel_y - y_baseline) < 0.4f;

            if (accel_stable && gyro_stable && y_stable && dt_since_detection > 2000)
            {
                fall_state = FALL_STATE_NORMAL;
                last_confidence = 0.0f;
                ESP_LOGI(TAG, "Returned to normal state");
            }
            break;
        }
    }
}

// Main fall detection task
static void fall_detection_task(void* arg)
{
    int16_t raw_accel[3], raw_gyro[3];
    sensor_data_t data;

    while (1)
    {
        if (mpu6050_read_raw(raw_accel, raw_gyro))
        {
            convert_raw_to_physical(raw_accel, raw_gyro, &data);
            analyze_fall_software(&data);

            // Debug info every 2 seconds
            static uint32_t last_debug = 0;
            if (data.timestamp - last_debug > 2000)
            {
                ESP_LOGI(TAG, "State: %d, Accel: %.2f, Gyro: %.2f, Confidence: %.1f%%",
                         fall_state, data.accel_magnitude, data.gyro_magnitude, last_confidence);
                last_debug = data.timestamp;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz sampling
    }
}

// Initialize and start fall detection system
void mpu_manager_start(void)
{
    ESP_LOGI(TAG, "Starting fall detection system...");

    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());

    // Initialize MPU6050
    mpu6050_init();

    // Calibrate gravity
    if (!calibrate_gravity())
    {
        ESP_LOGW(TAG, "Using default gravity baseline");
    }

    // Record initial orientation
    if (!record_initial_orientation())
    {
        ESP_LOGW(TAG, "Using default initial orientation");
    }

    ESP_LOGI(TAG, "Starting fall detection task...");

    // Create fall detection task
    xTaskCreate(fall_detection_task,
                "fall_detection",
                4096,
                NULL,
                5,
                NULL);
}
