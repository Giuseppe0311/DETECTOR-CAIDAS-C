#include "ble_adv.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mpu_manager.h"
#include "wifi_manager.h"


void app_main(void)
{
    // ESP_ERROR_CHECK(i2c_master_init());
    // mpu6050_init();
    wifi_manager_init();
    ble_adv_init("CUIDADO ABUELITO");
    // mpu_manager_start();
}
