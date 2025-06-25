//
// Created by barre on 6/25/2025.
//

#include "mpu_manager.h"
#include <stdio.h>
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

static const char* TAG = "MPU6050";

// Inicializa el I2C como maestro
esp_err_t i2c_master_init(void)
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

// Escribe un único byte en un registro del MPU6050
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                      (uint8_t[]){reg, data}, 2, pdMS_TO_TICKS(1000));
}

// Lee varios bytes consecutivos del MPU6050
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t* buf, size_t len)
{
    // primero envía la dirección de inicio
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR,
        &reg, 1, pdMS_TO_TICKS(1000)));
    // luego lee los datos
    return i2c_master_read_from_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                       buf, len, pdMS_TO_TICKS(1000));
}

// Configura el MPU6050 a 1kHz y despierta del modo sleep
void mpu6050_init(void)
{
    // Despertar (clear sleep bit)
    mpu6050_write_reg(PWR_MGMT_1_REG, 0x00);
    // Sample rate = Gyro output rate / (1 + SMPLRT_DIV)
    mpu6050_write_reg(SMPLRT_DIV_REG, 0x07); // 1kHz/(1+7) = 125Hz
    // DLPF configura a 42Hz bandwidth
    mpu6050_write_reg(CONFIG_REG, 0x03);
    // Gyro full scale ±250°/s
    mpu6050_write_reg(GYRO_CONFIG_REG, 0x00);
    // Accel full scale ±2g
    mpu6050_write_reg(ACCEL_CONFIG_REG, 0x00);

    ESP_LOGI(TAG, "MPU6050 inicializado");
}

// Lee y convierte los datos crudos a enteros de 16 bits
void mpu6050_read_raw(int16_t* accel, int16_t* gyro)
{
    uint8_t buf[14];
    // Lee acelerómetro (6), temperatura (2), giroscopio (6)
    if (mpu6050_read_regs(ACCEL_XOUT_H, buf, 14) == ESP_OK)
    {
        accel[0] = (buf[0] << 8) | buf[1];
        accel[1] = (buf[2] << 8) | buf[3];
        accel[2] = (buf[4] << 8) | buf[5];
        gyro[0] = (buf[8] << 8) | buf[9];
        gyro[1] = (buf[10] << 8) | buf[11];
        gyro[2] = (buf[12] << 8) | buf[13];
    }
    else
    {
        ESP_LOGW(TAG, "Error leyendo datos del MPU6050");
    }
}

static void mpu6050_task(void *arg)
{
    int16_t accel[3], gyro[3];
    while (1) {
        mpu6050_read_raw(accel, gyro);
        ESP_LOGI(TAG, "ACCEL  X=%6d  Y=%6d  Z=%6d", accel[0], accel[1], accel[2]);
        ESP_LOGI(TAG, "GYRO   X=%6d  Y=%6d  Z=%6d", gyro[0],  gyro[1],  gyro[2]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// 2) Arrancas el task tras inicializar el sensor
void mpu_manager_start(void)
{
    // asume que ya has llamado a i2c_master_init() y mpu6050_init()
    xTaskCreate(mpu6050_task,
                "mpu6050_task",
                2048,
                NULL,
                5,
                NULL);
}