idf_component_register(SRCS "cuidado_abu.c" "ble_adv.c" "wifi_manager.c"
        INCLUDE_DIRS "."
        REQUIRES json bt nvs_flash
)
