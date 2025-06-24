#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H
#include <stdbool.h>

typedef enum
{
    WIFI_CMD_SCAN,
    WIFI_CMD_CONNECT,
    WIFI_CMD_DISCONNECT,
    WIFI_CMD_GET_STATUS
} wifi_command_type_t;


typedef struct
{
    wifi_command_type_t cmd;
    char ssid[32];
    char password[64];
    bool (*response_callback)(const char* response);
} wifi_command_t;

void wifi_manager_init(void);
bool wifi_manager_send_command(const wifi_command_t* cmd);
void wifi_manager_set_ble_response_callback(bool (*send_ble_func)(const char*));

#endif
