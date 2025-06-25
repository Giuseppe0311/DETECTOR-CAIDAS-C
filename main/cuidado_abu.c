#include "ble_adv.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mpu_manager.h"
#include "wifi_manager.h"


void app_main(void)
{
    wifi_manager_init();
    ble_adv_init("CUIDADO ABUELITO");
    mpu_manager_start();
}
