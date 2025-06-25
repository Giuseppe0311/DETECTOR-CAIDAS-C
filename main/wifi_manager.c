#include "wifi_manager.h"

#include <string.h>
#include <stdio.h>
#include <time.h>

#include "cJSON.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_http_client.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_crt_bundle.h"

static const char* TAG = "WIFI_MGR";
static QueueHandle_t wifi_cmd_queue = NULL;
static TaskHandle_t wifi_task_handle = NULL;

static char device_uuid[37] = {0};

// Guardamos el último comando para usar su callback en los handlers
static wifi_command_t last_cmd;

// IDs de los registros de eventos
static esp_event_handler_instance_t wifi_event_inst;
static esp_event_handler_instance_t ip_event_inst;

// Variable para controlar si debemos hacer test de conectividad
static bool should_test_connectivity = false;

// URL del servidor para reportar status
static const char* DEVICE_STATUS_URL = "https://oldalert-server.vercel.app/api/device-status";

// Función para generar UUID v4
static void generate_uuid_v4(char* uuid_str)
{
    uint8_t uuid_bytes[16];

    // Generar 16 bytes aleatorios
    esp_fill_random(uuid_bytes, 16);

    // Configurar bits de versión (versión 4)
    uuid_bytes[6] = (uuid_bytes[6] & 0x0F) | 0x40;

    // Configurar bits de variante (variante RFC)
    uuid_bytes[8] = (uuid_bytes[8] & 0x3F) | 0x80;

    // Formatear como string
    snprintf(uuid_str, 37,
             "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
             uuid_bytes[0], uuid_bytes[1], uuid_bytes[2], uuid_bytes[3],
             uuid_bytes[4], uuid_bytes[5], uuid_bytes[6], uuid_bytes[7],
             uuid_bytes[8], uuid_bytes[9], uuid_bytes[10], uuid_bytes[11],
             uuid_bytes[12], uuid_bytes[13], uuid_bytes[14], uuid_bytes[15]);
}

// Función para obtener o generar UUID del dispositivo
static void init_device_uuid(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Abrir NVS
    err = nvs_open("device", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error abriendo NVS: %s", esp_err_to_name(err));
        return;
    }

    // Intentar leer UUID existente
    size_t uuid_len = sizeof(device_uuid);
    err = nvs_get_str(nvs_handle, "uuid", device_uuid, &uuid_len);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        // UUID no existe, generar uno nuevo
        generate_uuid_v4(device_uuid);

        // Guardar en NVS
        err = nvs_set_str(nvs_handle, "uuid", device_uuid);
        if (err == ESP_OK)
        {
            err = nvs_commit(nvs_handle);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "UUID generado y guardado: %s", device_uuid);
            }
            else
            {
                ESP_LOGE(TAG, "Error guardando UUID: %s", esp_err_to_name(err));
            }
        }
        else
        {
            ESP_LOGE(TAG, "Error escribiendo UUID: %s", esp_err_to_name(err));
        }
    }
    else if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "UUID cargado desde NVS: %s", device_uuid);
    }
    else
    {
        ESP_LOGE(TAG, "Error leyendo UUID: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
}

// Función para enviar status del dispositivo al servidor
static bool send_device_status(const char* ip_address)
{
    ESP_LOGI(TAG, "Enviando status del dispositivo al servidor...");

    // Crear JSON del payload
    cJSON* json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "device_id", device_uuid);
    cJSON_AddStringToObject(json, "ip", ip_address);

    char* json_string = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);

    if (!json_string)
    {
        ESP_LOGE(TAG, "Error creando JSON para device status");
        return false;
    }

    ESP_LOGI(TAG, "Payload JSON: %s", json_string);
    ESP_LOGI(TAG, "Payload length: %d", strlen(json_string));

    // Configurar cliente HTTP
    esp_http_client_config_t config = {
        .url = DEVICE_STATUS_URL,
        .method = HTTP_METHOD_POST,  // Especificar método explícitamente
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .use_global_ca_store = true,
        .disable_auto_redirect = false,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Error inicializando cliente HTTP para device status");
        free(json_string);
        return false;
    }

    // Configurar headers más explícitamente
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "User-Agent", "ESP32-Device");

    // Configurar el cuerpo de la petición
    esp_http_client_set_post_field(client, json_string, strlen(json_string));

    ESP_LOGI(TAG, "Enviando petición a: %s", DEVICE_STATUS_URL);

    // Realizar petición
    esp_err_t err = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_get_content_length(client);

    // Leer respuesta del servidor para debug
    char response_buffer[512] = {0};
    if (content_length > 0 && content_length < sizeof(response_buffer)) {
        int data_read = esp_http_client_read_response(client, response_buffer, sizeof(response_buffer) - 1);
        if (data_read > 0) {
            response_buffer[data_read] = '\0';
            ESP_LOGI(TAG, "Respuesta del servidor: %s", response_buffer);
        }
    }

    esp_http_client_cleanup(client);
    free(json_string);

    if (err == ESP_OK && (status_code >= 200 && status_code < 300))
    {
        ESP_LOGI(TAG, "Device status enviado exitosamente (HTTP %d)", status_code);
        return true;
    }
    else
    {
        ESP_LOGW(TAG, "Error enviando device status: err=%s, status=%d, content_length=%d",
                 esp_err_to_name(err), status_code, content_length);
        return false;
    }
}
static const char* reason_str(int reason)
{
    switch (reason)
    {
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_AUTH_EXPIRE:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        return "wrong_password";
    case WIFI_REASON_NO_AP_FOUND: return "network_not_found";
    case WIFI_REASON_ASSOC_FAIL: return "association_failed";
    case WIFI_REASON_BEACON_TIMEOUT: return "signal_lost";
    default: return "unknown";
    }
}

// Función para testear conectividad a internet
static bool test_internet_connectivity(void)
{
    ESP_LOGI(TAG, "Testeando conectividad a internet...");

    esp_http_client_config_t config = {
        .url = "http://httpbin.org/get",
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
        .disable_auto_redirect = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Error inicializando cliente HTTP");
        return false;
    }

    esp_err_t err = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err == ESP_OK && status_code == 200)
    {
        ESP_LOGI(TAG, "Test de conectividad exitoso");
        return true;
    }
    ESP_LOGW(TAG, "Test de conectividad falló: err=%s, status=%d",
             esp_err_to_name(err), status_code);
    return false;
}

// Función alternativa usando ping DNS
static bool test_dns_connectivity(void)
{
    ESP_LOGI(TAG, "Testeando conectividad DNS...");

    esp_http_client_config_t config = {
        .url = "http://8.8.8.8",
        .method = HTTP_METHOD_HEAD,
        .timeout_ms = 3000,
        .disable_auto_redirect = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        return false;
    }

    esp_err_t err = esp_http_client_perform(client);
    esp_http_client_cleanup(client);

    // Cualquier respuesta indica que hay conectividad
    return (err == ESP_OK);
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
             "{\"status\":\"error\",\"detail\":\"%s\"}",
             reason_str(d->reason));

    if (last_cmd.response_callback)
    {
        last_cmd.response_callback(buf);
    }

    should_test_connectivity = false;
}

// Handler: IP obtenida (conexión exitosa)
static void ip_got_ip_handler(void* arg,
                              esp_event_base_t base,
                              int32_t event_id,
                              void* event_data)
{
    ip_event_got_ip_t* ip_info = event_data;
    ESP_LOGI(TAG, "¡IP obtenida! IP: " IPSTR, IP2STR(&ip_info->ip_info.ip));

    // Marcamos que debemos testear conectividad
    should_test_connectivity = true;
}

// Tarea para testear conectividad (se ejecuta después de obtener IP)
static void connectivity_test_task(void* pvParameters)
{
    while (1)
    {
        if (should_test_connectivity)
        {
            should_test_connectivity = false;

            // Esperamos un poco para que la conexión se estabilice
            vTaskDelay(pdMS_TO_TICKS(2000));

            bool has_internet = test_internet_connectivity();

            if (!has_internet)
            {
                ESP_LOGW(TAG, "Test HTTP falló, probando DNS...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                has_internet = test_dns_connectivity();
            }

            char response[200];
            if (has_internet)
            {
                // Obtener IP actual para reportar al servidor
                esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                esp_netif_ip_info_t ip_info;

                if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
                {
                    char ip_str[16];
                    sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));

                    // Enviar status del dispositivo al servidor
                    bool status_sent = send_device_status(ip_str);

                    if (status_sent)
                    {
                        snprintf(response, sizeof(response),
                                 "{\"status\":\"success\",\"detail\":\"connected\",\"ip\":\"%s\",\"device_id\":\"%s\",\"server_notified\":true}",
                                 ip_str, device_uuid);
                    }
                    else
                    {
                        snprintf(response, sizeof(response),
                                 "{\"status\":\"success\",\"detail\":\"connected\",\"ip\":\"%s\",\"device_id\":\"%s\",\"server_notified\":false}",
                                 ip_str, device_uuid);
                    }
                }
                else
                {
                    snprintf(response, sizeof(response),
                             "{\"status\":\"success\",\"detail\":\"connected\",\"device_id\":\"%s\"}",
                             device_uuid);
                }

                ESP_LOGI(TAG, "WiFi conectado con acceso a internet");
            }
            else
            {
                snprintf(response, sizeof(response),
                         "{\"status\":\"error\",\"detail\":\"no_internet\"}");
                ESP_LOGW(TAG, "WiFi conectado sin acceso a internet");
            }

            if (last_cmd.response_callback)
            {
                last_cmd.response_callback(response);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void wifi_connect_task(const wifi_command_t* cmd)
{
    memcpy(&last_cmd, cmd, sizeof(wifi_command_t));

    ESP_LOGI(TAG, "Iniciando conexión a '%s'...", cmd->ssid);

    // 1. Verificar si ya hay una conexión activa
    wifi_ap_record_t current_ap;
    bool is_connected = (esp_wifi_sta_get_ap_info(&current_ap) == ESP_OK);

    if (is_connected)
    {
        ESP_LOGI(TAG, "Desconectando de red actual: '%s'", (char*)current_ap.ssid);

        // Desconectar de forma segura
        esp_err_t disconnect_err = esp_wifi_disconnect();
        if (disconnect_err != ESP_OK)
        {
            ESP_LOGW(TAG, "Error en desconexión: %s", esp_err_to_name(disconnect_err));
        }

        // Esperar a que la desconexión se complete
        // El evento WIFI_EVENT_STA_DISCONNECTED se disparará
        ESP_LOGI(TAG, "Esperando desconexión completa...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Dar tiempo para la desconexión

        // Verificar que realmente se desconectó
        int retry_count = 0;
        while (esp_wifi_sta_get_ap_info(&current_ap) == ESP_OK && retry_count < 5)
        {
            ESP_LOGI(TAG, "Aún conectado, esperando... (intento %d)", retry_count + 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            retry_count++;
        }

        if (retry_count >= 5)
        {
            ESP_LOGW(TAG, "Timeout esperando desconexión, continuando...");
        }
        else
        {
            ESP_LOGI(TAG, "Desconexión completada exitosamente");
        }
    }
    else
    {
        ESP_LOGI(TAG, "No hay conexión activa, procediendo directamente...");
    }

    // 2. Configurar nueva red
    ESP_LOGI(TAG, "Configurando nueva red: '%s'", cmd->ssid);
    wifi_config_t wifi_cfg = {0};
    strncpy((char*)wifi_cfg.sta.ssid, cmd->ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char*)wifi_cfg.sta.password, cmd->password, sizeof(wifi_cfg.sta.password) - 1);

    // Configuraciones adicionales para mejorar la conexión
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    esp_err_t config_err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (config_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configurando WiFi: %s", esp_err_to_name(config_err));
        if (cmd->response_callback)
        {
            cmd->response_callback("{\"status\":\"error\",\"detail\":\"config_failed\"}");
        }
        return;
    }

    // 3. Intentar conexión
    ESP_LOGI(TAG, "Iniciando conexión a '%s'...", cmd->ssid);
    esp_err_t connect_err = esp_wifi_connect();
    if (connect_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error iniciando conexión: %s", esp_err_to_name(connect_err));
        if (cmd->response_callback)
        {
            cmd->response_callback("{\"status\":\"error\",\"detail\":\"connect_failed\"}");
        }
        return;
    }

    ESP_LOGI(TAG, "Comando de conexión enviado, esperando eventos...");
    // Los eventos WIFI_EVENT_STA_DISCONNECTED o IP_EVENT_STA_GOT_IP
    // manejarán el resultado final
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
                should_test_connectivity = false;
                // send_ble_response("{\"status\":\"disconnected\"}");
                break;

            case WIFI_CMD_GET_STATUS:
                // Enviar estado actual con UUID del dispositivo
                char status_response[100];
                snprintf(status_response, sizeof(status_response),
                         "{\"status\":\"idle\",\"device_id\":\"%s\"}", device_uuid);
                if (cmd.response_callback)
                {
                    cmd.response_callback(status_response);
                }
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
    // Inicializar UUID del dispositivo
    init_device_uuid();
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

    // Crear tarea para test de conectividad
    xTaskCreate(connectivity_test_task, "conn_test", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "WiFi Manager inicializado con device_id: %s", device_uuid);
}
