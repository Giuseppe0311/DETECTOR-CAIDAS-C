#include "ble_adv.h"
#include "wifi_manager.h"


void app_main(void)
{
    wifi_manager_init();
    ble_adv_init("CUIDADO ABUELITO");
}
