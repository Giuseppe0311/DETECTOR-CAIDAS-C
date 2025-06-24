#ifndef BLE_ADV_H
#define BLE_ADV_H

#include <stdbool.h>
#include <stdint.h>

void ble_adv_init(const char* device_name);
bool ble_adv_is_connected(void);
uint16_t ble_adv_get_conn_id(void);
bool ble_adv_send_data(const char* data);

#endif
