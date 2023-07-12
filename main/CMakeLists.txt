set(MAIN_REQUIRES
    esp_secure_cert_mgr
    ble_module
    wifi_module
    sensor_module
    sub_pub_unsub
)

idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${MAIN_REQUIRES})

