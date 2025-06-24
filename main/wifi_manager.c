//
// Created by barre on 6/24/2025.
//
#include "wifi_manager.h"

#include <string.h>

#include "cJSON.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nvs_flash.h"


static const char* TAG = "WIFI_MGR";
static QueueHandle_t wifi_cmd_queue = NULL;
static TaskHandle_t wifi_task_handle = NULL;

// Guardamos el último comando para usar su callback en los handlers
static wifi_command_t last_cmd;

// IDs de los registros de eventos
static esp_event_handler_instance_t wifi_event_inst;
static esp_event_handler_instance_t ip_event_inst;

static const char* reason_str(int reason)
{
    switch (reason)
    {
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return "wrong_password";
    case WIFI_REASON_NO_AP_FOUND: return "network_not_found";
    case WIFI_REASON_ASSOC_FAIL: return "association_failed";
    case WIFI_REASON_BEACON_TIMEOUT: return "signal_lost";
    default: return "unknown";
    }
}

// Handler: desconexión WiFi (fallo de conexión)
static void wifi_disconnect_handler(void* arg,
                                    esp_event_base_t base,
                                    int32_t event_id,
                                    void* event_data)
{
    wifi_event_sta_disconnected_t* d = event_data;
    ESP_LOGW(TAG, "STA desconectado, reason: %d", d->reason);

    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"status\":\"connect_failed\",\"reason\":\"%s\"}",
             reason_str(d->reason));

    if (last_cmd.response_callback)
    {
        last_cmd.response_callback(buf);
    }
}

// Handler: IP obtenida (conexión exitosa)
static void ip_got_ip_handler(void* arg,
                              esp_event_base_t base,
                              int32_t event_id,
                              void* event_data)
{
    ip_event_got_ip_t* ip_info = event_data;
    ESP_LOGI(TAG, "¡Conectado! IP: " IPSTR, IP2STR(&ip_info->ip_info.ip));

    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"status\":\"connected\",\"ip\":\"" IPSTR "\"}",
             IP2STR(&ip_info->ip_info.ip));

    if (last_cmd.response_callback)
    {
        last_cmd.response_callback(buf);
    }
}


static void wifi_connect_task(const wifi_command_t* cmd)
{
    memcpy(&last_cmd, cmd, sizeof(wifi_command_t));

    ESP_LOGI(TAG, "Conectando a '%s'...", cmd->ssid);
    esp_wifi_disconnect();

    wifi_config_t wifi_cfg = {0};
    strncpy((char*)wifi_cfg.sta.ssid, cmd->ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char*)wifi_cfg.sta.password, cmd->password, sizeof(wifi_cfg.sta.password) - 1);

    if (esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg) != ESP_OK)
    {
        cmd->response_callback("{\"error\":\"config_failed\"}");
        return;
    }

    if (esp_wifi_connect() != ESP_OK)
    {
        cmd->response_callback("{\"status\":\"connect_failed\"}");
        return;
    }

    if (cmd->response_callback)
    {
        cmd->response_callback("{\"status\":\"connecting\"}");
    }
}


bool wifi_manager_send_command(const wifi_command_t* cmd)
{
    if (!wifi_cmd_queue) return false;

    return xQueueSend(wifi_cmd_queue, cmd, pdMS_TO_TICKS(100)) == pdTRUE;
}

static void wifi_scan_task(const wifi_command_t* cmd)
{
    ESP_LOGI(TAG, "Iniciando escaneo WiFi...");

    // 1) Escaneo activo con parámetros por defecto
    const wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE
    };

    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK)
    {
        if (cmd->response_callback) cmd->response_callback("{\"error\":\"scan_failed\"}");
        return;
    }

    // 2) Obtener número de AP
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count == 0)
    {
        if (cmd->response_callback) cmd->response_callback("{\"networks\":[]}");
        return;
    }

    // 3) Leer registros
    wifi_ap_record_t* ap_records = malloc(ap_count * sizeof(wifi_ap_record_t));
    if (!ap_records)
    {
        if (cmd->response_callback) cmd->response_callback("{\"error\":\"no_memory\"}");
        return;
    }
    esp_wifi_scan_get_ap_records(&ap_count, ap_records);

    // 4) Montar JSON con claves cortas
    cJSON* root = cJSON_CreateObject();
    cJSON* arr = cJSON_CreateArray();
    for (int i = 0; i < ap_count && i < 10; ++i)
    {
        cJSON* n = cJSON_CreateObject();
        cJSON_AddStringToObject(n, "s", (const char*)ap_records[i].ssid);
        cJSON_AddNumberToObject(n, "r", ap_records[i].rssi);
        cJSON_AddStringToObject(n, "a",
                                ap_records[i].authmode == WIFI_AUTH_OPEN ? "o" : "x");
        cJSON_AddItemToArray(arr, n);
    }
    cJSON_AddItemToObject(root, "w", arr);

    // 5) Serializar sin formato
    char* json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    free(ap_records);
    if (!json_str)
    {
        if (cmd->response_callback) cmd->response_callback("{\"error\":\"json_failed\"}");
        return;
    }

    // 6) Enviar en trozos (chunks) de 200 bytes
    const size_t CHUNK_SIZE = 200;
    const size_t total_len = strlen(json_str);
    for (size_t offset = 0; offset < total_len; offset += CHUNK_SIZE)
    {
        size_t len_chunk = total_len - offset;
        if (len_chunk > CHUNK_SIZE) len_chunk = CHUNK_SIZE;
        char chunk[CHUNK_SIZE + 1];
        memcpy(chunk, json_str + offset, len_chunk);
        chunk[len_chunk] = '\0';
        ESP_LOGI(TAG, "Chunk [%u..%u]: %s", (unsigned)offset, (unsigned)(offset + len_chunk), chunk);
        if (cmd->response_callback)
        {
            bool ok = cmd->response_callback(chunk);
            if (!ok)
            {
                ESP_LOGE(TAG, "Fallo al enviar chunk BLE");
                break;
            }
        }
    }
    ESP_LOGI(TAG, "Escaneo WiFi completado, %d redes encontradas", ap_count);
    // 7) Liberar JSON original
    free(json_str);
}

static void wifi_manager_task(void* pvParameters)
{
    wifi_command_t cmd;

    while (1)
    {
        if (xQueueReceive(wifi_cmd_queue, &cmd, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Procesando comando WiFi: %d", cmd.cmd);

            switch (cmd.cmd)
            {
            case WIFI_CMD_SCAN:
                ESP_LOGI(TAG, "Escaneando redes WiFi...");
                wifi_scan_task(&cmd);
                break;

            case WIFI_CMD_CONNECT:
                ESP_LOGI(TAG, "Conectando a: %s", cmd.ssid);
                wifi_connect_task(&cmd);
                break;

            case WIFI_CMD_DISCONNECT:
                esp_wifi_disconnect();
                // send_ble_response("{\"status\":\"disconnected\"}");
                break;

            case WIFI_CMD_GET_STATUS:
                // Enviar estado actual
                // send_ble_response("{\"status\":\"idle\"}");
                break;
            }
        }
    }
}

void wifi_manager_init(void)
{
    wifi_cmd_queue = xQueueCreate(10, sizeof(wifi_command_t));
    if (!wifi_cmd_queue)
    {
        ESP_LOGE(TAG, "Error creando cola WiFi");
        return;
    }
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
        &wifi_disconnect_handler, NULL, &wifi_event_inst);

    esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &ip_got_ip_handler, NULL, &ip_event_inst);

    // Crear tarea WiFi
    xTaskCreate(wifi_manager_task, "wifi_mgr", 4096, NULL, 5, &wifi_task_handle);

    ESP_LOGI(TAG, "WiFi Manager inicializado");
}
