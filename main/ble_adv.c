#include "ble_adv.h"
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "wifi_manager.h"

static const char* TAG = "BLE_ADV";
static bool s_connected = false;
static uint16_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_conn_id = 0;

// UUID del servicio UART Nordic (igual que MicroPython)
#define SERVICE_UUID        0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
#define CHAR_UUID_RX        0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
#define CHAR_UUID_TX        0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E

// Handles para las características
static uint16_t service_handle;
static uint16_t char_tx_handle;
static uint16_t char_rx_handle;

// Estado de creación de características
static int char_creation_step = 0;

// Función para procesar comandos recibidos por BLE
static void process_ble_command(const char* command, const size_t len)
{
    char cmd_str[len + 1];
    memcpy(cmd_str, command, len);
    cmd_str[len] = '\0';

    ESP_LOGI(TAG, "Comando recibido: %s", cmd_str);

    // Parsear JSON
    cJSON* json = cJSON_Parse(cmd_str);
    if (!json)
    {
        ble_adv_send_data("{\"error\":\"invalid_json\"}");
        return;
    }

    cJSON* action = cJSON_GetObjectItem(json, "a");
    if (!action || !cJSON_IsString(action))
    {
        ble_adv_send_data("{\"error\":\"missing_action\"}");
        cJSON_Delete(json);
        return;
    }

    wifi_command_t wifi_cmd = {0};

    if (strcmp(action->valuestring, "s") == 0)
    {
        wifi_cmd.cmd = WIFI_CMD_SCAN;
        wifi_cmd.response_callback = ble_adv_send_data;
        wifi_manager_send_command(&wifi_cmd);
    }
    else if (strcmp(action->valuestring, "c") == 0)
    {
        const cJSON* ssid = cJSON_GetObjectItem(json, "ssid");
        const cJSON* password = cJSON_GetObjectItem(json, "password");

        if (ssid && cJSON_IsString(ssid))
        {
            wifi_cmd.cmd = WIFI_CMD_CONNECT;
            wifi_cmd.response_callback = ble_adv_send_data;
            strncpy(wifi_cmd.ssid, ssid->valuestring, sizeof(wifi_cmd.ssid) - 1);

            if (password && cJSON_IsString(password))
            {
                strncpy(wifi_cmd.password, password->valuestring, sizeof(wifi_cmd.password) - 1);
            }

            wifi_manager_send_command(&wifi_cmd);
        }
        else
        {
            ble_adv_send_data("{\"error\":\"missing_ssid\"}");
        }
    }
    else if (strcmp(action->valuestring, "wifi_status") == 0)
    {
        wifi_cmd.cmd = WIFI_CMD_GET_STATUS;
        wifi_manager_send_command(&wifi_cmd);
    }
    else
    {
        ble_adv_send_data("{\"error\":\"unknown_action\"}");
    }

    cJSON_Delete(json);
}


// Callback de eventos GATT Server
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t* param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATT Server registrado");
        s_gatts_if = gatts_if;

        // Crear servicio
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id.inst_id = 0,
            .id.uuid.len = ESP_UUID_LEN_128,
            .id.uuid.uuid.uuid128 = {SERVICE_UUID}
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, 8); // Más handles para descriptores
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Servicio creado, handle: %d", param->create.service_handle);
        service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(service_handle);

        // Crear primera característica RX (escritura)
        esp_bt_uuid_t char_rx_uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid = {.uuid128 = {CHAR_UUID_RX}}
        };

        esp_attr_value_t char_rx_val = {
            .attr_max_len = 20,
            .attr_len = 0,
            .attr_value = NULL
        };

        esp_attr_control_t ctrl = {
            .auto_rsp = ESP_GATT_AUTO_RSP
        };

        esp_ble_gatts_add_char(service_handle,
                               &char_rx_uuid,
                               ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_WRITE,
                               &char_rx_val,
                               &ctrl);
        char_creation_step = 1;

        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "Característica añadida, handle: %d, step: %d", param->add_char.attr_handle, char_creation_step);

        if (char_creation_step == 1)
        {
            // Se creó RX
            char_rx_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Característica RX añadida, handle: %d", char_rx_handle);

            // Crear característica TX (notificación)
            esp_bt_uuid_t char_tx_uuid = {
                .len = ESP_UUID_LEN_128,
                .uuid = {.uuid128 = {CHAR_UUID_TX}}
            };

            esp_attr_value_t char_tx_val = {
                .attr_max_len = 20,
                .attr_len = 0,
                .attr_value = NULL
            };

            esp_ble_gatts_add_char(service_handle, &char_tx_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   &char_tx_val, NULL);
            char_creation_step = 2;
        }
        else if (char_creation_step == 2)
        {
            // Se creó TX
            char_tx_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "Característica TX añadida, handle: %d", char_tx_handle);

            // Añadir descriptor CCCD para TX (necesario para notificaciones)
            esp_bt_uuid_t cccd_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
            };

            uint8_t cccd_val[2] = {0x00, 0x00};
            esp_attr_value_t cccd_attr_val = {
                .attr_max_len = 2,
                .attr_len = 2,
                .attr_value = cccd_val
            };

            esp_attr_control_t cccd_ctrl = {
                .auto_rsp = ESP_GATT_AUTO_RSP // <-- ¡igual que en RX!
            };

            esp_ble_gatts_add_char_descr(service_handle, &cccd_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                         &cccd_attr_val, &cccd_ctrl);
            char_creation_step = 3;
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Cliente conectado, conn_id: %d", param->connect.conn_id);
        s_connected = true;
        s_conn_id = param->connect.conn_id;
        esp_ble_gap_stop_advertising();
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Cliente desconectado, conn_id: %d", param->disconnect.conn_id);
        s_connected = false;
        s_conn_id = 0;
        esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x20,
            .adv_int_max = 0x40,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        const uint16_t h = param->write.handle;
        if (h == char_rx_handle)
        {
            process_ble_command((char*)param->write.value, param->write.len);
        }
        break;


    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "Servicio iniciado");
        break;

    default:
        break;
    }
}

// Callback de eventos GAP
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t* param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Advertising iniciado correctamente");
        }
        else
        {
            ESP_LOGE(TAG, "Error al iniciar advertising: %d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising detenido");
        break;

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Datos de advertising configurados");
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan response configurado");
        esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x20, // 20ms
            .adv_int_max = 0x40, // 40ms
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

void ble_adv_init(const char* device_name)
{
    ESP_LOGI(TAG, "Inicializando BLE con nombre: %s", device_name);

    // 1) NVS para BLE
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2) Inicializar controlador BT
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // 3) Inicializar Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // 4) Registrar callbacks
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));

    // 5) Registrar aplicación GATT
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    // 6) Configurar nombre de dispositivo
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(device_name));

    // 7) Configurar datos de advertising con el UUID del servicio
    uint8_t service_uuid128[] = {SERVICE_UUID};

    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = false, // Nombre va en scan response
        .include_txpower = true, // Incluir TX power
        .min_interval = 0x0006, // 3.75ms
        .max_interval = 0x0010, // 10ms
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 16, // UUID de 128 bits
        .p_service_uuid = service_uuid128,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    // 8) Configurar scan response con el nombre
    esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp = true,
        .include_name = true,
        .include_txpower = false,
        .min_interval = 0x0000,
        .max_interval = 0x0000,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = 0,
    };

    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data));
}

bool ble_adv_is_connected(void)
{
    return s_connected;
}

uint16_t ble_adv_get_conn_id(void)
{
    return s_connected ? s_conn_id : 0;
}

// Función para enviar datos por TX (notificaciones)
bool ble_adv_send_data(const char* data)
{
    if (!s_connected || s_gatts_if == ESP_GATT_IF_NONE)
    {
        return false;
    }

    // Delimitadores y tamaño de chunk
    const char *start_delim = "<<START>>";
    const char *end_delim   = "<<END>>";
    const size_t CHUNK_SIZE = 20;

    size_t start_len   = strlen(start_delim);
    size_t data_len    = strlen(data);
    size_t end_len     = strlen(end_delim);
    size_t wrapped_len = start_len + data_len + end_len;

    // Construir el mensaje completo: <<START>> + data + <<END>>
    char *wrapped = malloc(wrapped_len + 1);
    if (wrapped == NULL)
    {
        return false;
    }
    memcpy(wrapped,                    start_delim, start_len);
    memcpy(wrapped + start_len,        data,        data_len);
    memcpy(wrapped + start_len + data_len, end_delim,   end_len);
    wrapped[wrapped_len] = '\0';

    for (size_t offset = 0; offset < wrapped_len; offset += CHUNK_SIZE)
    {
        size_t len_chunk = wrapped_len - offset;
        if (len_chunk > CHUNK_SIZE) len_chunk = CHUNK_SIZE;

        esp_err_t err = esp_ble_gatts_send_indicate(
            s_gatts_if,
            s_conn_id,
            char_tx_handle,
            len_chunk,
            (uint8_t*)(wrapped + offset),
            false
        );
        if (err != ESP_OK)
        {
            free(wrapped);
            return false;
        }
    }

    free(wrapped);
    return true;
}
