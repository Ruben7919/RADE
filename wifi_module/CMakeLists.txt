idf_component_register(SRCS "wifi_module.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_netif
                       REQUIRES ble_module esp_wifi nvs_flash)
